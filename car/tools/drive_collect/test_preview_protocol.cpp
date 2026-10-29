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
#include <memory>
#include <stdexcept>
#include <sys/file.h>
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
    auto shared = std::unique_ptr<PreviewShared>(new PreviewShared{});
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
    check(!writePreviewFrame(*shared, cv::Mat(2, 2, CV_8UC1), 0), "nhận ảnh sai định dạng");
    check(!writePreviewFrame(*shared, cv::Mat(721, 1280, CV_8UC3), 0), "tràn vùng ảnh");
    PreviewState state;
    std::strcpy(state.state, "CHAY"); state.cmd_speed = 5; state.cmd_steer = -10;
    state.images = 77; state.recording = false; state.frame_age_ms = 42;
    std::strcpy(state.stop_reason, "key_s"); shared->state.write(state);
    PreviewState snapshot;
    check(shared->state.read(snapshot) && snapshot.images == 77 && snapshot.cmd_speed == 5 &&
          snapshot.cmd_steer == -10 && !snapshot.recording && snapshot.frame_age_ms == 42 &&
          std::strcmp(snapshot.state, "CHAY") == 0 && std::strcmp(snapshot.stop_reason, "key_s") == 0,
          "sai ảnh chụp trạng thái");
}
void child(const char* mode, PreviewChildStatus expected, bool shown) {
    check(setenv("UCR_DRIVE_FAKE_PREVIEW", mode, 1) == 0, "setenv");
    PreviewHost host;
    check(host.start() == PreviewStartResult::Started, "không fork được con giả");
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(1, 2, 3));
    check(host.publishFrame(frame, nowMs()), "không giao được khung đầu");
    until([&] { return host.pollChild() == expected; }, "sai trạng thái con");
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
    until([&] { received += host.pollKeys(); return received.size() >= 11; }, "thiếu phím pipe");
    check(received == "rywadc xEs3", "sai giải mã phím");
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
}
int main() {
    try {
        options(); seqlock(); frames();
        child("ok", PreviewChildStatus::Shown, true);
        child("fail_start", PreviewChildStatus::Died, false);
        child("die_after_frame", PreviewChildStatus::Died, true);
        child("close_after_frame", PreviewChildStatus::Closed, true);
        keys(); hangAndTiming(); disabledAndLock(); noZombies();
        std::cout << "PASS preview_protocol (không cần X)\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
