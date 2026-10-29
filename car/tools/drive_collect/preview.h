#pragma once

#include <opencv2/core.hpp>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

namespace drive {

struct PreviewOptions {
    bool no_preview = false;
    bool require_preview = false;
    double preview_scale = 0.5;
    int preview_fps = 10;
};
enum class PreviewOptionResult { Unhandled, Parsed, Error };
inline bool validatePreviewOptions(const PreviewOptions& options, std::string& error) {
    if (options.no_preview && options.require_preview)
        error = "--no-preview không dùng cùng --require-preview (mã 2)";
    else if (!std::isfinite(options.preview_scale) || options.preview_scale < 0.2 || options.preview_scale > 1.0)
        error = "--preview-scale phải trong [0.2, 1.0] (mã 2)";
    else if (options.preview_fps < 1 || options.preview_fps > 30)
        error = "--preview-fps phải là số nguyên trong [1, 30] (mã 2)";
    else { error.clear(); return true; }
    return false;
}
// Gọi cho từng tham số trước bộ phân tích cũ; Parsed đã tăng index nếu có giá trị.
inline PreviewOptionResult parsePreviewOption(int argc, const char* const argv[], int& index,
                                             PreviewOptions& options, std::string& error) {
    if (index < 0 || index >= argc) return PreviewOptionResult::Unhandled;
    const std::string key(argv[index]);
    PreviewOptions candidate = options;
    if (key == "--no-preview") candidate.no_preview = true;
    else if (key == "--require-preview") candidate.require_preview = true;
    else if (key == "--preview-scale" || key == "--preview-fps") {
        if (index + 1 >= argc) { error = "Thiếu giá trị " + key; return PreviewOptionResult::Error; }
        const char* value = argv[index + 1];
        char* end = nullptr;
        errno = 0;
        const double number = key == "--preview-fps" ? double(std::strtol(value, &end, 10))
                                                     : std::strtod(value, &end);
        if (errno || end == value || *end || !std::isfinite(number) ||
            (key == "--preview-fps" && (number < 1 || number > 30))) {
            error = "Giá trị không hợp lệ cho " + key; return PreviewOptionResult::Error;
        }
        if (key == "--preview-scale") candidate.preview_scale = number;
        else candidate.preview_fps = static_cast<int>(number);
        if (!validatePreviewOptions(candidate, error)) return PreviewOptionResult::Error;
        options = candidate; ++index;
        return PreviewOptionResult::Parsed;
    } else return PreviewOptionResult::Unhandled;
    if (!validatePreviewOptions(candidate, error)) return PreviewOptionResult::Error;
    options = candidate;
    return PreviewOptionResult::Parsed;
}

struct PreviewState {
    char state[24] = "KHOA";
    double cmd_speed = 0;
    double cmd_steer = 0;
    uint64_t images = 0;
    bool recording = true;
    double frame_age_ms = 0;
    char stop_reason[64] = {};
};

// Seqlock (khóa bằng số thứ tự): một bên ghi, bên đọc thử đúng một lần.
// Từng byte nguyên tử tránh tranh chấp dữ liệu C++ khi số thứ tự đang lẻ.
template <class T> struct PreviewSlot {
    static_assert(std::is_trivially_copyable<T>::value, "Can du lieu sao chep duoc");
    std::atomic<uint32_t> sequence{0};
    std::atomic<unsigned char> bytes[sizeof(T)]{};
    void write(const T& value) noexcept {
        sequence.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        const auto* source = reinterpret_cast<const unsigned char*>(&value);
        for (size_t i = 0; i < sizeof(T); ++i) bytes[i].store(source[i], std::memory_order_relaxed);
        sequence.fetch_add(1, std::memory_order_release);
    }
    bool read(T& value) const noexcept {
        const auto before = sequence.load(std::memory_order_acquire);
        if (before & 1) return false;
        T copy{};
        auto* target = reinterpret_cast<unsigned char*>(&copy);
        for (size_t i = 0; i < sizeof(T); ++i) target[i] = bytes[i].load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (before != sequence.load(std::memory_order_relaxed)) return false;
        value = copy;
        return true;
    }
};

constexpr int kPreviewMaxWidth = 1280;
constexpr int kPreviewMaxHeight = 720;
constexpr size_t kPreviewMaxBytes = size_t(kPreviewMaxWidth) * kPreviewMaxHeight * 3;
struct PreviewFrameHeader {
    uint32_t width = 0, height = 0;
    int64_t captured_ms = 0;
};
struct PreviewShared {
    std::atomic<uint32_t> frame_sequence{0};
    std::atomic<unsigned char> header[sizeof(PreviewFrameHeader)]{};
    // Không dùng {}: khởi tạo mảng 2,7 triệu phần tử atomic làm GCC ngốn >6 GB RAM ở pha phân tích cú pháp.
    // Vùng nhớ đến từ mmap nên đã bằng 0; đối tượng PreviewShared phải tạo bằng default-init (không dùng {}).
    std::atomic<unsigned char> pixels[kPreviewMaxBytes];
    PreviewSlot<PreviewState> state;
    std::atomic<uint32_t> ready{0}, shown{0}, stop{0};
};
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_CHAR_LOCK_FREE == 2,
              "Bo nho chia se can nguyen tu khong dung khoa");
struct PreviewFrame {
    cv::Mat bgr;
    int64_t captured_ms = 0;
    uint32_t sequence = 0;
};
bool writePreviewFrame(PreviewShared& shared, const cv::Mat& bgr, int64_t captured_ms) noexcept;
bool readPreviewFrame(const PreviewShared& shared, PreviewFrame& frame);

enum class PreviewChildStatus { NotReady, Shown, Closed, Died };
enum class PreviewStartResult { Disabled, Started, Unavailable };

class PreviewHost {
public:
    explicit PreviewHost(PreviewOptions options = {});
    ~PreviewHost();
    PreviewHost(const PreviewHost&) = delete;
    PreviewHost& operator=(const PreviewHost&) = delete;
    // Bắt buộc gọi trước mọi luồng, camera, Motor::Init; start chỉ gọi một lần.
    PreviewStartResult start();
    // Một nguồn camera; khung đã giao phải giữ bất biến (camera cấp bộ đệm mới).
    // Chỉ giao tham chiếu, bỏ khung nếu hộp thư đang bận; không đợi tiến trình con.
    bool publishFrame(const cv::Mat& bgr, int64_t captured_ms);
    // Một nguồn trạng thái ở vòng điều khiển, độc lập với bên ghi ảnh.
    void publishState(const PreviewState& state) noexcept;
    std::string pollKeys(); // Tối đa 64 byte; E = Esc, 3 = Ctrl+C.
    PreviewChildStatus pollChild() noexcept;
    bool wasShown() const noexcept;
    bool ready() const noexcept;
    const std::string& diagnostic() const noexcept { return diagnostic_; }
    // Chỉ gọi sau motor 0/servo 0; chờ con có hạn rồi SIGKILL, thu hồi waitpid.
    void shutdown() noexcept;
private:
    void feed();
    void unavailable(const std::string& reason);
    PreviewOptions options_;
    PreviewShared* shared_ = nullptr;
    int child_ = -1, keys_ = -1;
    bool started_ = false, shown_ = false;
    PreviewChildStatus status_ = PreviewChildStatus::NotReady;
    std::string diagnostic_;
    std::atomic<bool> feeding_{false};
    std::atomic<bool> feeder_failed_{false};
    std::thread feeder_;
    std::mutex mailbox_mutex_;
    std::condition_variable mailbox_changed_;
    cv::Mat mailbox_;
    int64_t captured_ms_ = 0;
    std::chrono::steady_clock::time_point next_frame_{};
};

// Chỉ dùng trong nhánh con sau fork; không gọi ở cha.
int runPreviewChild(PreviewShared& shared, int key_fd);

} // namespace drive
