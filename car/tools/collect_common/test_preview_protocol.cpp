#include "preview.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace drive;
namespace {
using Clock = std::chrono::steady_clock;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template <class F> void until(F condition, const char* message, int timeout_ms = 3000) {
    const auto end = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!condition()) {
        check(Clock::now() < end, message);
        std::this_thread::yield();
    }
}
int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
void windowTracking() {
    const double exception = std::numeric_limits<double>::quiet_NaN();
    auto calibrate = [](PreviewWindowTracker& tracker, double autosize, double visible) {
        for (int ms = 0; ms <= 100; ms += 10) {
            check(!tracker.sample(ms, autosize, visible), "báo đóng khi đang hiệu chuẩn");
            if (ms < 100) check(tracker.property() == PreviewWindowProperty::None,
                                "chọn thuộc tính trước 100 ms");
        }
    };
    PreviewWindowTracker gtk;
    calibrate(gtk, 1, -1);
    check(gtk.property() == PreviewWindowProperty::Autosize && !gtk.warning(), "không chọn AUTOSIZE GTK");
    for (int ms = 101; ms <= 10100; ++ms)
        check(!gtk.sample(ms, 1, -1), "VISIBLE không hỗ trợ làm tự đóng");
    check(!gtk.sample(10101, -1, -1) && gtk.pendingClose(), "đóng ngay lần lỗi đầu");
    check(!gtk.sample(10250, -1, -1), "báo đóng trước 150 ms");
    check(gtk.sample(10251, -1, -1) && gtk.lastValue() == -1, "không đóng tại 150 ms");

    PreviewWindowTracker unsupported;
    calibrate(unsupported, -1, -1);
    check(unsupported.warning() && unsupported.property() == PreviewWindowProperty::None, "thiếu cờ cảnh báo");
    for (int ms = 101; ms <= 10100; ++ms)
        check(!unsupported.sample(ms, -1, -1), "thuộc tính không hỗ trợ vẫn báo đóng");
    check(!unsupported.sample(11000, 1, 1) && unsupported.warning(), "tự chọn lại sau hiệu chuẩn thất bại");

    PreviewWindowTracker minimized;
    calibrate(minimized, 1, 1);
    check(minimized.property() == PreviewWindowProperty::Autosize, "không ưu tiên AUTOSIZE");
    for (int ms = 101; ms <= 10100; ++ms)
        check(!minimized.sample(ms, 1, ms % 2), "thu nhỏ/mất focus làm tự đóng");
    check(!minimized.sample(11000, -1, 1) && minimized.sample(11150, -1, 1),
          "đổi sang VISIBLE khi AUTOSIZE đóng");

    PreviewWindowTracker errors;
    calibrate(errors, 1, -1);
    check(!errors.sample(110, exception, -1), "ngoại lệ đơn làm đóng");
    check(!errors.sample(120, 1, -1) && !errors.pendingClose(), "không xóa hạn sau phục hồi");
    check(!errors.sample(1000, 1, -1), "ngoại lệ cũ còn gây đóng");
    check(!errors.sample(1010, exception, -1), "ngoại lệ mới làm đóng ngay");
    check(!errors.sample(1159, exception, -1), "ngoại lệ đóng trước 150 ms");
    check(errors.sample(1160, exception, -1) && std::isnan(errors.lastValue()), "ngoại lệ dài không đóng");

    PreviewWindowTracker flicker;
    calibrate(flicker, 1, -1);
    check(!flicker.sample(110, 0, -1) && !flicker.sample(259, -1, -1), "chập chờn đóng quá sớm");
    check(!flicker.sample(260, 1, -1) && !flicker.pendingClose(), "phục hồi không xóa chuỗi lỗi");
    check(!flicker.sample(270, 0, -1) && !flicker.sample(419, 0, -1) && flicker.sample(420, 0, -1),
          "cộng dồn hai khoảng lỗi rời nhau");

    PreviewWindowTracker visible;
    calibrate(visible, -1, 1);
    check(visible.property() == PreviewWindowProperty::Visible, "không chọn VISIBLE dự phòng");
    check(!visible.sample(110, 1, 0) && visible.sample(260, 1, 0), "không theo VISIBLE đã chọn");

    PreviewWindowTracker tooFew;
    for (int n = 0; n < 9; ++n) check(!tooFew.sample(n * 100, 1, -1), "hiệu chuẩn ít mẫu");
    check(tooFew.property() == PreviewWindowProperty::None, "chọn khi chưa đủ 10 mẫu");
    check(!tooFew.sample(900, 1, -1) && tooFew.property() == PreviewWindowProperty::Autosize, "không chọn ở mẫu 10");

    PreviewWindowTracker unstable;
    check(!unstable.sample(0, 0, 1), "mẫu đầu chập chờn");
    for (int ms = 10; ms <= 100; ms += 10) check(!unstable.sample(ms, 1, 1), "hiệu chuẩn chập chờn");
    check(unstable.property() == PreviewWindowProperty::Visible, "chọn thuộc tính từng lỗi khi hiệu chuẩn");
    PreviewWindowTracker allErrors;
    calibrate(allErrors, exception, exception);
    check(allErrors.warning() && !allErrors.sample(10000, exception, exception), "ngoại lệ lúc hiệu chuẩn gây đóng");
    PreviewWindowTracker earlyClose;
    for (int ms = 0; ms <= 20; ms += 10) check(!earlyClose.sample(ms, 1, -1), "đóng khi còn mở");
    check(!earlyClose.sample(30, -1, -1) && earlyClose.pendingClose(), "mất cờ đóng trong hiệu chuẩn");
    for (int ms = 40; ms <= 170; ms += 10)
        check(!earlyClose.sample(ms, -1, -1) && earlyClose.pendingClose(), "mất chuỗi đóng sớm/đóng trước hạn");
    check(earlyClose.sample(180, -1, -1) && earlyClose.pendingClose(), "không đóng tại 150 ms từ lỗi sớm");
    PreviewWindowTracker bothSupported;
    check(!bothSupported.sample(0, 1, 1), "báo đóng mẫu hỗ trợ đầu");
    for (int ms = 10; ms <= 150; ms += 10)
        check(!bothSupported.sample(ms, -1, 1) && bothSupported.pendingClose(), "thuộc tính còn mở che chuỗi đóng sớm");
    check(bothSupported.sample(160, -1, 1), "đổi thuộc tính làm mất chuỗi đóng trong hiệu chuẩn");
    PreviewWindowTracker sparseClose;
    check(!sparseClose.sample(0, 1, -1) && !sparseClose.sample(10, -1, -1) && sparseClose.pendingClose(),
          "không theo dõi đóng khi chưa đủ 10 mẫu");
    check(sparseClose.sample(160, -1, -1), "chưa đủ mẫu hiệu chuẩn làm mất phát hiện đóng");
    std::cout << "PASS window_tracking: GTK/thuộc tính lỗi/thu nhỏ/ngoại lệ/mốc 100-150 ms\n";
}
void options() {
    PreviewOptions value;
    std::string error;
    check(value.preview_scale == 0.5 && value.preview_fps == 10 && !value.no_preview && !value.require_preview,
          "sai mặc định preview");
    auto parse = [&](const char* key, const char* argument) {
        const char* argv[] = {"test", key, argument}; int index = 1;
        return parsePreviewOption(argument ? 3 : 2, argv, index, value, error);
    };
    check(parse("--preview-scale", "0.2") == PreviewOptionResult::Parsed, "scale 0.2");
    check(parse("--preview-scale", "1.0") == PreviewOptionResult::Parsed, "scale 1.0");
    for (const char* invalid : {"0.199", "1.001", "nan", "inf", "x", "0.5x", ""})
        check(parse("--preview-scale", invalid) == PreviewOptionResult::Error, "nhận scale sai");
    check(parse("--preview-fps", "1") == PreviewOptionResult::Parsed, "fps 1");
    check(parse("--preview-fps", "30") == PreviewOptionResult::Parsed, "fps 30");
    for (const char* invalid : {"0", "31", "10.5", "1.0", "1e1", "1e30", "nan", "2x", ""})
        check(parse("--preview-fps", invalid) == PreviewOptionResult::Error, "nhận fps sai");
    check(parse("--preview-fps", nullptr) == PreviewOptionResult::Error, "thiếu fps");
    check(parse("--pipeline", nullptr) == PreviewOptionResult::Unhandled, "chiếm tham số cũ");
    check(parse("--no-preview", nullptr) == PreviewOptionResult::Parsed, "no-preview");
    check(parse("--require-preview", nullptr) == PreviewOptionResult::Error, "nhận cặp cờ xung đột");
    value = PreviewOptions{};
    check(parse("--require-preview", nullptr) == PreviewOptionResult::Parsed, "require-preview");
    check(parse("--no-preview", nullptr) == PreviewOptionResult::Error, "xung đột thứ tự ngược");
}
void seqlock() {
    struct Pair { uint64_t number = 0, inverse = ~uint64_t(0); };
    PreviewSlot<Pair> slot;
    slot.write(Pair{});
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (uint64_t n = 1; n <= 100000; ++n) slot.write(Pair{n, ~n});
        done.store(true);
    });
    bool consistent = true;
    size_t reads = 0;
    do {
        Pair pair;
        if (slot.read(pair)) { ++reads; if (pair.inverse != ~pair.number) consistent = false; }
    } while (!done.load());
    writer.join();
    Pair last;
    check(slot.read(last) && last.number == 100000, "mất bản cuối seqlock");
    check(consistent, "đọc trộn hai lần ghi seqlock");
    slot.sequence.fetch_add(1);
    check(!slot.read(last), "bên đọc chờ/nhận số lẻ");
    slot.sequence.fetch_add(1);
    std::cout << "seqlock: " << reads << " lần đọc\n";
}
void frames() {
    auto shared = std::unique_ptr<PreviewShared>(new PreviewShared);
    cv::Mat original(72, 128, CV_8UC3, cv::Scalar(12, 34, 56));
    // Kiểm cả ảnh không liền bộ đệm.
    cv::Mat roi = original(cv::Rect(1, 1, 61, 33));
    check(writePreviewFrame(*shared, roi, 1234), "không ghi được ROI");
    PreviewFrame frame;
    check(readPreviewFrame(*shared, frame), "không đọc được ảnh");
    check(frame.bgr.cols == 61 && frame.bgr.rows == 33 && frame.captured_ms == 1234, "sai tiêu đề ảnh");
    check(cv::norm(frame.bgr, roi, cv::NORM_INF) == 0, "sai dữ liệu ảnh");
    check(!readPreviewFrame(*shared, frame), "đọc lại khung cũ");
    shared->frame_sequence.fetch_add(1);
    PreviewFrame other;
    check(!readPreviewFrame(*shared, other), "đọc khung đang ghi");
    shared->frame_sequence.fetch_add(1);
    shared->frame_busy.store(1);
    check(!writePreviewFrame(*shared, roi, 2000), "bên ghi chờ/ghi đè khi bộ đệm ảnh bận");
    check(!readPreviewFrame(*shared, other), "bên đọc vượt khóa bộ đệm ảnh");
    shared->frame_busy.store(0);
    check(writePreviewFrame(*shared, roi, 2000) && readPreviewFrame(*shared, other), "không nhả khóa ảnh sau sao chép");
    check(!writePreviewFrame(*shared, cv::Mat(2, 2, CV_8UC1), 0), "nhận ảnh sai định dạng");
    check(!writePreviewFrame(*shared, cv::Mat(721, 1280, CV_8UC3), 0), "tràn vùng ảnh");
    PreviewState state;
    std::strcpy(state.state, "CHAY_TIEN"); state.cmd_speed = 5; state.cmd_steer = -10;
    state.images = 77; state.recording = false; state.frame_age_ms = 42;
    std::strcpy(state.stop_reason, "reverse_conflict"); shared->state.write(state);
    PreviewState snapshot;
    check(shared->state.read(snapshot) && snapshot.images == 77 && snapshot.cmd_speed == 5 &&
          snapshot.cmd_steer == -10 && !snapshot.recording && snapshot.frame_age_ms == 42 &&
          std::strcmp(snapshot.state, "CHAY_TIEN") == 0 && std::strcmp(snapshot.stop_reason, "reverse_conflict") == 0,
          "sai ảnh chụp trạng thái");
    std::strcpy(state.state, "CHAY_LUI"); state.cmd_speed = -3;
    shared->state.write(state);
    check(shared->state.read(snapshot) && snapshot.cmd_speed == -3 && std::strcmp(snapshot.state, "CHAY_LUI") == 0,
          "mất dấu tốc độ hoặc trạng thái lùi khi truyền sang cửa sổ");
    shared->monitor.write(PreviewMonitor{true, PreviewWindowProperty::Visible, -1, 1});
    PreviewMonitor monitor;
    check(shared->monitor.read(monitor) && monitor.calibrated && monitor.autosize == -1 && monitor.visible == 1 &&
          std::strcmp(previewMonitorName(monitor.property), "VISIBLE") == 0, "mất chẩn đoán hiệu chuẩn giữa con và cha");
}
void child(const char* mode, PreviewChildStatus expected, bool shown) {
    check(setenv("UCR_DRIVE_FAKE_PREVIEW", mode, 1) == 0, "setenv");
    PreviewHost host;
    check(host.start() == PreviewStartResult::Started, "không fork được con giả");
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(1, 2, 3));
    check(host.publishFrame(frame, nowMs()), "không giao được khung đầu");
    const auto end = Clock::now() + std::chrono::seconds(3);
    while (host.pollChild() != expected) {
        check(Clock::now() < end, (std::string("sai trạng thái con (chế độ ") + mode + ", có " +
              std::to_string(int(host.pollChild())) + ", cần " + std::to_string(int(expected)) + ")").c_str());
        std::this_thread::yield();
    }
    check(host.wasShown() == shown, "sai cờ shown");
    if (!shown) check(!host.diagnostic().empty(), "thiếu hướng dẫn lỗi khởi động");
    const auto before = Clock::now();
    host.shutdown(); host.shutdown();
    check(Clock::now() - before < std::chrono::seconds(1), "shutdown quá hạn");
}
void keys() {
    setenv("UCR_DRIVE_FAKE_PREVIEW", "keys:RYwAdc xEs3p", 1);
    PreviewHost host;
    check(host.start() == PreviewStartResult::Started, "khởi động keys");
    std::string received;
    until([&] { received += host.pollKeys(); return received.size() >= 9; }, "thiếu phím pipe");
    check(received == "wadc xEs3", "sai giải mã phím hoặc R/Y chưa bị bỏ qua");
}
void hangAndTiming() {
    setenv("UCR_DRIVE_FAKE_PREVIEW", "hang", 1);
    PreviewHost host;
    check(host.start() == PreviewStartResult::Started, "khởi động hang");
    until([&] { return host.ready(); }, "con hang chưa ready");
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(3, 4, 5));
    double maximum_us = 0;
    int offered = 0;
    const auto end = Clock::now() + std::chrono::milliseconds(500);
    while (Clock::now() < end) {
        const auto start = Clock::now();
        if (host.publishFrame(frame, nowMs())) ++offered;
        maximum_us = std::max(maximum_us, std::chrono::duration<double, std::micro>(Clock::now() - start).count());
        check(host.pollChild() == PreviewChildStatus::NotReady && !host.wasShown(), "hang lại hiển thị");
        std::this_thread::yield();
    }
    check(offered >= 1 && offered <= 6, "sai giới hạn fps/hộp thư");
    std::cout << "mock 1280x720/hang: giao " << offered << " khung/500ms; publishFrame max="
              << maximum_us << "us (gồm trễ bộ lập lịch)\n";
    host.shutdown();
}
void disabledAndLock() {
    PreviewOptions options; options.no_preview = true;
    PreviewHost disabled(options);
    check(disabled.start() == PreviewStartResult::Disabled && !disabled.ready(), "no-preview vẫn fork");
    check(disabled.pollKeys().empty(), "no-preview có phím");
    char name[] = "/tmp/ucr_preview_lock_XXXXXX";
    const int fd = mkstemp(name);
    check(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0, "tạo khóa riêng");
    setenv("UCR_DRIVE_FAKE_PREVIEW", "hang", 1);
    PreviewHost host;
    check(host.start() == PreviewStartResult::Started, "khởi động kiểm khóa");
    until([&] { return host.ready(); }, "chưa đóng fd thừa hưởng");
    ::close(fd);
    const int second = open(name, O_RDWR);
    const bool released = second >= 0 && flock(second, LOCK_EX | LOCK_NB) == 0;
    if (second >= 0) ::close(second);
    unlink(name);
    check(released, "con giữ khóa xe thừa hưởng");
}
void noZombies() {
    int status = 0; errno = 0;
    check(waitpid(-1, &status, WNOHANG) == -1 && errno == ECHILD, "còn con/zombie sau shutdown");
}
void childWithFullStderr() {
    void* memory = mmap(nullptr, sizeof(PreviewShared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    check(memory != MAP_FAILED, "mmap kiểm stderr");
    auto* shared = new (memory) PreviewShared;
    int pipe_fd[2];
    const int made = pipe2(pipe_fd, O_NONBLOCK | O_CLOEXEC);
    if (made != 0) { shared->~PreviewShared(); munmap(memory, sizeof(PreviewShared)); }
    check(made == 0, "pipe kiểm stderr");
    char bytes[4096] = {};
    while (::write(pipe_fd[1], bytes, sizeof(bytes)) > 0) {}
    while (::write(pipe_fd[1], bytes, 1) > 0) {}
    const bool full = errno == EAGAIN || errno == EWOULDBLOCK;
    const bool blocking = fcntl(pipe_fd[1], F_SETFL, fcntl(pipe_fd[1], F_GETFL) & ~O_NONBLOCK) == 0;
    setenv("UCR_DRIVE_FAKE_PREVIEW", "close_after_frame", 1);
    shared->frame_sequence.store(2);
    const pid_t pid = full && blocking ? fork() : -1;
    if (pid == 0) {
        if (dup2(pipe_fd[1], STDERR_FILENO) < 0 || dup2(pipe_fd[1], STDOUT_FILENO) < 0) _exit(99);
        ::close(pipe_fd[0]); ::close(pipe_fd[1]);
        _exit(runPreviewChild(*shared, -1));
    }
    int status = 0;
    pid_t result = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (pid > 0 && (result = waitpid(pid, &status, WNOHANG)) == 0 && Clock::now() < deadline)
        std::this_thread::yield();
    const bool exited = result == pid && pid > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 10;
    if (pid > 0 && result == 0) {
        ::kill(pid, SIGKILL);
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    }
    PreviewMonitor monitor;
    const bool diagnosed = shared->monitor.read(monitor) && monitor.closed &&
                           monitor.property == PreviewWindowProperty::Autosize && monitor.close_value == -1 &&
                           shared->window_closed.load() == 1;
    ::close(pipe_fd[0]); ::close(pipe_fd[1]);
    shared->~PreviewShared(); munmap(memory, sizeof(PreviewShared));
    check(full && blocking && pid > 0, "không tạo được tiền đề stderr đầy/chặn");
    check(exited && diagnosed, "con kẹt stderr hoặc mất chẩn đoán đóng qua vùng chia sẻ");
}
}
int main() {
    try {
        windowTracking(); options(); seqlock(); frames();
        child("ok", PreviewChildStatus::Shown, true);
        child("fail_start", PreviewChildStatus::Died, false);
        child("die_after_frame", PreviewChildStatus::Died, true);
        child("close_after_frame", PreviewChildStatus::Closed, true);
        childWithFullStderr(); keys(); hangAndTiming(); disabledAndLock(); noZombies();
        std::cout << "PASS preview_protocol (không cần X)\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
