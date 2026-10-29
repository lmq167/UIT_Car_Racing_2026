#include "preview.h"
#include <opencv2/imgproc.hpp>
#if defined(UCR_DRIVE_HAVE_HIGHGUI) && (!defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS)
#include <opencv2/highgui.hpp>
#endif
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <new>
#include <poll.h>
#include <sstream>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace drive {
namespace {
using Clock = std::chrono::steady_clock;
[[maybe_unused]] int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
char decodeKey(int key) {
    if (key < 0) return 0;
    if (key == 27) return 'E';
    if (key == 3) return '3';
    if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
    return key < 128 && std::strchr("wasdcxry ", key) && key != 0 ? char(key) : 0;
}
[[maybe_unused]] void sendKey(int fd, int key) {
    const char decoded = decodeKey(key);
    if (decoded) { const auto ignored = ::write(fd, &decoded, 1); (void)ignored; }
}
// Bỏ cả khóa xe thừa hưởng; con chỉ giữ đầu ghi phím và stdout/stderr.
void closeInherited(int keep) {
    DIR* directory = opendir("/proc/self/fd");
    if (directory) {
        const int scan = dirfd(directory);
        while (auto* entry = readdir(directory)) {
            char* end = nullptr;
            const long fd = std::strtol(entry->d_name, &end, 10);
            if (end != entry->d_name && *end == 0 && fd >= 3 && fd != keep && fd != scan)
                ::close(static_cast<int>(fd));
        }
        closedir(directory);
    } else {
        const long limit = sysconf(_SC_OPEN_MAX);
        for (long fd = 3; fd < limit; ++fd) if (fd != keep) ::close(static_cast<int>(fd));
    }
    ::close(STDIN_FILENO);
}
}

bool writePreviewFrame(PreviewShared& shared, const cv::Mat& bgr, int64_t captured_ms) noexcept {
    if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols > kPreviewMaxWidth || bgr.rows > kPreviewMaxHeight)
        return false;
    PreviewFrameHeader header{};
    header.width = bgr.cols; header.height = bgr.rows; header.captured_ms = captured_ms;
    shared.frame_sequence.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    const auto* bytes = reinterpret_cast<const unsigned char*>(&header);
    for (size_t i = 0; i < sizeof(header); ++i) shared.header[i].store(bytes[i], std::memory_order_relaxed);
    const size_t row_bytes = size_t(bgr.cols) * 3;
    for (int y = 0; y < bgr.rows; ++y) {
        const auto* row = bgr.ptr<unsigned char>(y);
        for (size_t x = 0; x < row_bytes; ++x)
            shared.pixels[size_t(y) * row_bytes + x].store(row[x], std::memory_order_relaxed);
    }
    shared.frame_sequence.fetch_add(1, std::memory_order_release);
    return true;
}
bool readPreviewFrame(const PreviewShared& shared, PreviewFrame& frame) {
    const auto sequence = shared.frame_sequence.load(std::memory_order_acquire);
    if (sequence == 0 || (sequence & 1) || sequence == frame.sequence) return false;
    PreviewFrameHeader header{};
    auto* bytes = reinterpret_cast<unsigned char*>(&header);
    for (size_t i = 0; i < sizeof(header); ++i) bytes[i] = shared.header[i].load(std::memory_order_relaxed);
    if (!header.width || !header.height || header.width > kPreviewMaxWidth || header.height > kPreviewMaxHeight)
        return false;
    cv::Mat copy(header.height, header.width, CV_8UC3);
    const size_t count = size_t(header.width) * header.height * 3;
    for (size_t i = 0; i < count; ++i) copy.data[i] = shared.pixels[i].load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (sequence != shared.frame_sequence.load(std::memory_order_relaxed)) return false;
    frame.bgr = std::move(copy); frame.captured_ms = header.captured_ms; frame.sequence = sequence;
    return true;
}

int runPreviewChild(PreviewShared& shared, int key_fd) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
    const char* env = std::getenv("UCR_DRIVE_FAKE_PREVIEW");
    const std::string mode = env ? env : "ok";
    if (mode == "fail_start") return 11;
    shared.ready.store(1, std::memory_order_release);
    if (mode == "hang") for (;;) ::pause();
    PreviewFrame frame;
    size_t key_index = 5;
    auto next_key = Clock::now();
    while (!shared.stop.load(std::memory_order_acquire)) {
        if (readPreviewFrame(shared, frame)) {
            shared.shown.store(1, std::memory_order_release);
            if (mode == "die_after_frame") { ::kill(getpid(), SIGKILL); return 12; }
            if (mode == "close_after_frame") return 10;
        }
        if (mode.compare(0, 5, "keys:") == 0 && key_index < mode.size() && Clock::now() >= next_key) {
            const char key = mode[key_index++];
            sendKey(key_fd, key == 'E' ? 27 : key == '3' ? 3 : key);
            next_key = Clock::now() + std::chrono::milliseconds(20);
        }
        ::poll(nullptr, 0, 2);
    }
    return 0;
#elif defined(UCR_DRIVE_HAVE_HIGHGUI)
    try {
        const char* window = "UCR drive collect";
        cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
        shared.ready.store(1, std::memory_order_release);
        PreviewFrame frame;
        PreviewState state{};
        auto next_redraw = Clock::now();
        while (!shared.stop.load(std::memory_order_acquire)) {
            const bool fresh = readPreviewFrame(shared, frame);
            if (!frame.bgr.empty() && (fresh || Clock::now() >= next_redraw)) {
                shared.state.read(state);
                state.state[sizeof(state.state) - 1] = 0;
                state.stop_reason[sizeof(state.stop_reason) - 1] = 0;
                cv::Mat display = frame.bgr.clone();
                std::ostringstream line;
                line << state.state << " v=" << state.cmd_speed << " steer=" << state.cmd_steer
                     << " images=" << state.images << " rec=" << (state.recording ? "ON" : "OFF");
                cv::putText(display, line.str(), cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(0, 255, 0), 1);
                line.str(""); line.clear();
                line << "age=" << std::max(state.frame_age_ms, double(std::max<int64_t>(0, nowMs() - frame.captured_ms)))
                     << "ms stop=" << state.stop_reason;
                cv::putText(display, line.str(), cv::Point(8, 40), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(0, 255, 255), 1);
                cv::imshow(window, display);
                sendKey(key_fd, cv::waitKey(1));
                shared.shown.store(1, std::memory_order_release);
                next_redraw = Clock::now() + std::chrono::milliseconds(100);
            } else sendKey(key_fd, cv::waitKey(1));
            if (shared.shown.load(std::memory_order_acquire)) {
                // Một số bộ hiển thị ném lỗi khi nút X đã hủy hẳn cửa sổ.
                try { if (cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) return 10; }
                catch (...) { return 10; }
            }
            ::poll(nullptr, 0, 2);
        }
        cv::destroyWindow(window);
        return 0;
    } catch (...) { return 11; }
#else
    (void)shared; (void)key_fd;
    return 11;
#endif
}

PreviewHost::PreviewHost(PreviewOptions options) : options_(options) {}
PreviewHost::~PreviewHost() { shutdown(); }
void PreviewHost::unavailable(const std::string& reason) {
    status_ = PreviewChildStatus::Died;
    diagnostic_ = reason + "; chạy sudo --preserve-env=DISPLAY,XAUTHORITY ... hoặc --no-preview";
    if (options_.require_preview) diagnostic_ += "; yêu cầu dừng rồi thoát 7 (preview_unavailable)";
}
PreviewStartResult PreviewHost::start() {
    if (started_) { unavailable("PreviewHost::start chỉ được gọi một lần"); return PreviewStartResult::Unavailable; }
    started_ = true;
    std::string error;
    if (!validatePreviewOptions(options_, error)) { unavailable(error); return PreviewStartResult::Unavailable; }
    if (options_.no_preview) return PreviewStartResult::Disabled;
#if !defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS
#ifndef UCR_DRIVE_HAVE_HIGHGUI
    unavailable("bản này không có HighGUI"); return PreviewStartResult::Unavailable;
#else
    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    if ((!display || !*display) && (!wayland || !*wayland)) {
        unavailable("không có DISPLAY / WAYLAND_DISPLAY"); return PreviewStartResult::Unavailable;
    }
#endif
#endif
    void* memory = mmap(nullptr, sizeof(PreviewShared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED) { unavailable("không tạo được bộ nhớ preview"); return PreviewStartResult::Unavailable; }
    shared_ = new (memory) PreviewShared{};
    shared_->state.write(PreviewState{});
    int pipe_fd[2];
    if (pipe2(pipe_fd, O_NONBLOCK | O_CLOEXEC) != 0) {
        shutdown(); unavailable("không tạo được ống phím preview"); return PreviewStartResult::Unavailable;
    }
    struct sigaction ignore{};
    ignore.sa_handler = SIG_IGN; sigemptyset(&ignore.sa_mask);
    if (sigaction(SIGPIPE, &ignore, nullptr) != 0) {
        ::close(pipe_fd[0]); ::close(pipe_fd[1]); shutdown();
        unavailable("không bỏ qua được SIGPIPE"); return PreviewStartResult::Unavailable;
    }
    const pid_t parent = getpid();
    child_ = fork();
    if (child_ == 0) {
        if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || getppid() != parent) _exit(11);
        for (int signal : {SIGINT, SIGTSTP, SIGQUIT, SIGHUP}) sigaction(signal, &ignore, nullptr);
        struct sigaction normal{}; normal.sa_handler = SIG_DFL; sigemptyset(&normal.sa_mask);
        sigaction(SIGTERM, &normal, nullptr);
        sigset_t empty; sigemptyset(&empty); sigprocmask(SIG_SETMASK, &empty, nullptr);
        closeInherited(pipe_fd[1]);
        int code = 11;
        try { code = runPreviewChild(*shared_, pipe_fd[1]); } catch (...) {}
        _exit(code);
    }
    ::close(pipe_fd[1]); keys_ = pipe_fd[0];
    if (child_ < 0) { shutdown(); unavailable("fork preview thất bại"); return PreviewStartResult::Unavailable; }
    feeding_.store(true);
    try { feeder_ = std::thread(&PreviewHost::feed, this); }
    catch (...) { shutdown(); unavailable("không tạo được luồng cấp preview"); return PreviewStartResult::Unavailable; }
    return PreviewStartResult::Started;
}
bool PreviewHost::publishFrame(const cv::Mat& bgr, int64_t captured_ms) {
    if (!feeding_.load(std::memory_order_relaxed) || bgr.empty() || bgr.type() != CV_8UC3) return false;
    const auto now = Clock::now();
    std::unique_lock<std::mutex> lock(mailbox_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || now < next_frame_) return false;
    mailbox_ = bgr; captured_ms_ = captured_ms;
    next_frame_ = now + std::chrono::microseconds(1000000 / options_.preview_fps);
    lock.unlock(); mailbox_changed_.notify_one();
    return true;
}
void PreviewHost::publishState(const PreviewState& state) noexcept {
    if (shared_) shared_->state.write(state);
}
void PreviewHost::feed() {
    try {
        while (feeding_.load()) {
            cv::Mat source;
            int64_t captured;
            {
                std::unique_lock<std::mutex> lock(mailbox_mutex_);
                mailbox_changed_.wait(lock, [this] { return !feeding_.load() || !mailbox_.empty(); });
                if (!feeding_.load()) break;
                source = std::move(mailbox_); captured = captured_ms_;
            }
            const double scale = std::min(options_.preview_scale,
                std::min(double(kPreviewMaxWidth) / source.cols, double(kPreviewMaxHeight) / source.rows));
            cv::Mat small;
            cv::resize(source, small, cv::Size(std::max(1, int(std::lround(source.cols * scale))),
                                             std::max(1, int(std::lround(source.rows * scale)))), 0, 0, cv::INTER_AREA);
            writePreviewFrame(*shared_, small, captured);
        }
    } catch (...) {
        // Lỗi xử lý preview chỉ làm con chết, không thoát cha hay ghi phần cứng.
        feeder_failed_.store(true, std::memory_order_release);
        feeding_.store(false);
    }
}
std::string PreviewHost::pollKeys() {
    char keys[64];
    if (keys_ < 0) return {};
    const auto count = ::read(keys_, keys, sizeof(keys));
    return count > 0 ? std::string(keys, size_t(count)) : std::string();
}
bool PreviewHost::wasShown() const noexcept {
    return shown_ || (shared_ && shared_->shown.load(std::memory_order_acquire));
}
bool PreviewHost::ready() const noexcept {
    return shared_ && shared_->ready.load(std::memory_order_acquire);
}
PreviewChildStatus PreviewHost::pollChild() noexcept {
    if (child_ <= 0) return status_;
    if (feeder_failed_.exchange(false, std::memory_order_acq_rel)) ::kill(child_, SIGKILL);
    shown_ = wasShown();
    int code = 0;
    const pid_t result = waitpid(child_, &code, WNOHANG);
    if (result == child_ || (result < 0 && errno != EINTR)) {
        child_ = -1;
        feeding_.store(false, std::memory_order_relaxed);
        mailbox_changed_.notify_one();
        status_ = result > 0 && WIFEXITED(code) && WEXITSTATUS(code) == 10
            ? PreviewChildStatus::Closed : PreviewChildStatus::Died;
        // Con có thể bật shown rồi chết giữa hai lần đọc của cha.
        shown_ = wasShown();
        if (!shown_) {
            try { unavailable("con preview không mở/hiển thị được cửa sổ hoặc chết trước khung đầu"); }
            catch (...) {}
        }
        return status_;
    }
    return shown_ ? PreviewChildStatus::Shown : PreviewChildStatus::NotReady;
}
void PreviewHost::shutdown() noexcept {
    feeding_.store(false);
    { std::lock_guard<std::mutex> lock(mailbox_mutex_); mailbox_changed_.notify_one(); }
    if (feeder_.joinable()) feeder_.join();
    if (shared_) { shown_ = wasShown(); shared_->stop.store(1, std::memory_order_release); }
    if (child_ > 0) {
        const auto deadline = Clock::now() + std::chrono::milliseconds(100);
        while (child_ > 0 && Clock::now() < deadline) {
            pollChild();
            if (child_ > 0) ::poll(nullptr, 0, 2);
        }
        if (child_ > 0) {
            ::kill(child_, SIGKILL);
            int code;
            while (waitpid(child_, &code, 0) < 0 && errno == EINTR) {}
            child_ = -1;
        }
    }
    if (keys_ >= 0) { ::close(keys_); keys_ = -1; }
    mailbox_.release();
    if (shared_) { shared_->~PreviewShared(); munmap(shared_, sizeof(PreviewShared)); shared_ = nullptr; }
}
} // namespace drive
