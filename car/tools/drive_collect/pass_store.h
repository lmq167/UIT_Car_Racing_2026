#pragma once

#include <cstdint>
#include <string>
#include <vector>

#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 8
#include <experimental/filesystem>
#else
#include <filesystem>
#endif

namespace drive_collect {
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 8
namespace pass_fs = std::experimental::filesystem;
#else
namespace pass_fs = std::filesystem;
#endif

enum class StoreError { None = 0, InvalidState = 2,
                        InvalidFrame = 3, Io = 4, Exhausted = 5 };

struct StoreStatus {
    StoreError code = StoreError::None;
    std::string message;
    explicit operator bool() const { return code == StoreError::None; }
};

struct SessionInfo {
    // -1: tự lấy thời gian Unix hiện tại, đơn vị ms.
    int64_t ts_ms = -1;
    std::string git;
    std::vector<std::string> argv;
    std::string label; // Giá trị --label của cả phiên, không phải nhãn lượt.
    std::string pipeline;
};

struct CaptureColumns {
    uint64_t frame = 0;
    int64_t monotonic_ns = 0;
    int width = 0, height = 0;
    double mean_luma = 0, lap_var = 0;
    uint64_t dropped_since_last = 0;
    double cmd_speed = 0;
    int cmd_steer = 0;
    std::string state;
};

// Xe đứng: cách ít nhất 2000 ms; xe chạy: caller giữ nhịp ghi hiện có.
// elapsed_ms lấy từ đồng hồ đơn điệu, tính từ lần ghi thành công gần nhất.
// Caller cho phép ảnh đầu tiên riêng; hàm này không giữ trạng thái hay ngủ chờ.
bool frameDue(bool stopped, int64_t elapsed_ms);

// Một chủ ghi duy nhất trong out-root; caller kiểm status() ngay sau khởi tạo.
// Mọi lỗi I/O được chốt: caller dừng xe, thoát mã 4; lớp này không điều khiển xe.
class PassStore {
public:
    explicit PassStore(pass_fs::path outRoot, const SessionInfo& session = {});
    PassStore(const PassStore&) = delete;
    PassStore& operator=(const PassStore&) = delete;

    const StoreStatus& status() const { return status_; }
    uint64_t nextPass() const { return next_; }
    uint64_t pass() const { return pass_; }
    bool active() const { return active_; }
    const std::string& lastFile() const { return lastFile_; }

    StoreStatus beginPass(int64_t ts_ms);
    // ts_ms cũng là wall_time_unix_ms; các cột capture khác do caller cung cấp.
    StoreStatus saveFrame(const std::vector<unsigned char>& jpeg, int64_t ts_ms,
                          const CaptureColumns& capture);
    StoreStatus endPass(const std::string& reason, int64_t ts_ms = -1);

private:
    pass_fs::path root_;
    StoreStatus status_;
    uint64_t next_ = 1, pass_ = 0, frames_ = 0;
    bool active_ = false;
    std::string lastFile_;

    StoreStatus ioError(const std::string& message);
    void append(const char* name, const char* header, const std::string& row);
    void replace(const pass_fs::path& target, const std::string& contents);
    void event(const char* kind, int64_t ts_ms, const std::string& reason);
    void initialize(const SessionInfo& session);
};
} // namespace drive_collect
