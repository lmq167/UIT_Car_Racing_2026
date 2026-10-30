#include "options.h"
#include "../collect_common/preview.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#if !defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS
#include "hardware/Motor.h"
#include "hardware/Servo.h"
#endif
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#ifndef UCR_DRIVE_GIT_REVISION
#define UCR_DRIVE_GIT_REVISION "unknown"
#endif
extern char** environ;
using namespace collect_drive;
using Clock = std::chrono::steady_clock;

namespace {
static_assert(std::atomic<int>::is_always_lock_free, "Co tin hieu phai khong khoa");
std::atomic<int> stopping{0};
void signalStop(int s) { stopping.store(s); }
int64_t monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
int64_t nowMs() { return monotonicNs() / 1000000; }
int64_t wallMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string utcNow() {
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{}; gmtime_r(&t, &tm);
    std::ostringstream out; out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}
struct Failure : std::runtime_error {
    int code;
    Failure(int c, const std::string& message) : std::runtime_error(message), code(c) {}
};
void warning() noexcept {
    const char text[] = "KIEM TRA XE, DUNG CONG TAC NGUON\n";
    pollfd out{STDERR_FILENO, POLLOUT, 0};
    if (poll(&out, 1, 0) > 0 && (out.revents & POLLOUT)) {
        const auto ignored = ::write(STDERR_FILENO, text, sizeof(text) - 1); (void)ignored;
    }
}
void trace(const char* event, const std::string& value = {}) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
    if (const char* path = std::getenv("UCR_DRIVE_FAKE_TRACE")) {
        const int fd = ::open(path, O_WRONLY | O_CREAT | O_APPEND | O_NONBLOCK | O_CLOEXEC, 0600);
        if (fd >= 0) {
            const auto line = std::to_string(nowMs()) + ' ' + event +
                (value.empty() ? "" : " " + value) + '\n';
            const auto ignored = ::write(fd, line.data(), line.size()); (void)ignored;
            ::close(fd);
        }
    }
#else
    (void)event; (void)value;
#endif
}
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
bool fakeOpenFail(const char* name) {
    const char* value = std::getenv("UCR_DRIVE_FAKE_OPEN_FAIL");
    return value && std::string(value) == name;
}
bool fakeMotorFail() {
    const char* value = std::getenv("UCR_DRIVE_FAKE_MOTOR_FAIL");
    return value && std::string(value) == "1";
}
#endif

class VehicleLock {
    int fd_ = -1;
public:
    VehicleLock() {
        const char* path = "/tmp/ucr_vehicle.lock";
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        if (const char* value = std::getenv("UCR_DRIVE_FAKE_LOCK")) path = value;
#endif
        fd_ = ::open(path, O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd_ < 0) throw Failure(1, "Không mở được khóa xe");
        if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            const int error = errno; ::close(fd_); fd_ = -1;
            throw Failure(error == EWOULDBLOCK || error == EAGAIN ? 6 : 1,
                          "Không lấy được khóa xe; không chạm motor");
        }
    }
    int fd() const { return fd_; }
    ~VehicleLock() { if (fd_ >= 0) ::close(fd_); }
};
struct Paths { fs::path root, stop; };
Paths executablePaths() {
    std::array<char, 4096> buf{};
    const auto n = readlink("/proc/self/exe", buf.data(), buf.size() - 1);
    if (n <= 0 || static_cast<size_t>(n) >= buf.size() - 1)
        throw Failure(1, "Không tìm được đường dẫn thật của tệp chạy");
    const fs::path executable(std::string(buf.data(), static_cast<size_t>(n)));
    auto root = executable.parent_path();
    while (!fs::is_regular_file(root / "config.yaml")) {
        const auto parent = root.parent_path();
        if (parent == root || parent.empty())
            throw Failure(1, "Không tìm thấy config.yaml khi đi lên từ tệp chạy");
        root = parent;
    }
    const auto stop = executable.parent_path() / "ucr_stop";
    if (!fs::is_regular_file(stop) || access(stop.c_str(), X_OK) != 0)
        throw Failure(1, "Thiếu ucr_stop thực thi được cạnh tệp chạy");
    return {root, stop};
}
// posix_spawn dùng được sau khi preview/ghi ảnh đã có luồng; không chạy shell.
bool externalStop(const Paths& paths) noexcept {
    try {
        const auto binary = paths.stop.string(), config = (paths.root / "config.yaml").string();
        char* args[] = {const_cast<char*>(binary.c_str()), const_cast<char*>("--config"),
                        const_cast<char*>(config.c_str()), nullptr};
        posix_spawnattr_t attr;
        if (posix_spawnattr_init(&attr) != 0) return false;
        sigset_t empty, defaults;
        sigemptyset(&empty); sigemptyset(&defaults);
        for (int s : {SIGINT, SIGTERM, SIGHUP, SIGPIPE}) sigaddset(&defaults, s);
        int error = posix_spawnattr_setsigmask(&attr, &empty);
        if (!error) error = posix_spawnattr_setsigdefault(&attr, &defaults);
        if (!error) error = posix_spawnattr_setpgroup(&attr, 0);
        if (!error) error = posix_spawnattr_setflags(&attr,
            POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETPGROUP);
        pid_t child = -1;
        if (!error) error = posix_spawn(&child, binary.c_str(), nullptr, &attr, args, environ);
        posix_spawnattr_destroy(&attr);
        if (error) return false;
        const auto deadline = nowMs() + kStopTimeoutMs;
        int status = 0;
        while (nowMs() < deadline) {
            const auto result = waitpid(child, &status, WNOHANG);
            if (result == child) return WIFEXITED(status) && WEXITSTATUS(status) == 0;
            if (result < 0 && errno != EINTR) return false;
            ::poll(nullptr, 0, kPollMs);
        }
        ::kill(-child, SIGTERM);
        const auto killAt = nowMs() + kStopKillMs;
        while (nowMs() < killAt) {
            const auto result = waitpid(child, &status, WNOHANG);
            if (result == child || (result < 0 && errno != EINTR)) return false;
            ::poll(nullptr, 0, kPollMs);
        }
        ::kill(-child, SIGKILL);
        // Không chờ vô hạn một tiến trình bị kẹt trong nhân hệ điều hành.
        const auto reapAt = nowMs() + kStopKillMs;
        while (nowMs() < reapAt) {
            const auto result = waitpid(child, &status, WNOHANG);
            if (result == child || (result < 0 && errno != EINTR)) break;
            ::poll(nullptr, 0, kPollMs);
        }
    } catch (...) {}
    return false;
}

std::atomic<int64_t> hardwareSince{0};
std::atomic<int> hardwareKind{0}; // 1 UART, 2 I2C, 3 Init nguyên bản BTC.
std::atomic<bool> hardwareFault{false};
struct HardwareCall {
    explicit HardwareCall(int kind) { hardwareKind = kind; hardwareSince = nowMs(); }
    ~HardwareCall() {
        hardwareSince = 0;
        // Khi watchdog đã nhận quyền dừng, không cho luồng cũ cấp lại lệnh chạy.
        while (hardwareFault) ::poll(nullptr, 0, kPollMs);
    }
};
struct DriveMotor {
    bool reverse;
#if !defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS
    Motor device;
    int readiness = -1;
    ~DriveMotor() { if (readiness >= 0) ::close(readiness); }
#endif
    explicit DriveMotor(bool r) : reverse(r) {}
    bool init() {
        HardwareCall call(3);
        trace("motor_init");
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        return !fakeOpenFail("motor") && !fakeOpenFail("motor_init");
#else
        const bool ok = device.Init(); // Một lần duy nhất, không thử lại.
        readiness = ::open("/dev/ttyTHS1", O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        return ok && !device.InitHadWriteFailure() && device.LastWriteOk() && readiness >= 0;
#endif
    }
    bool SetSpeedCms(float v) {
        HardwareCall call(1);
        if (hardwareFault) return false;
        const float command = applyMotorSign(clampHwSpeed(v), reverse);
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        const bool ok = !fakeMotorFail();
        trace("motor_speed", std::to_string(command) + (ok ? "" : " fail"));
        return ok;
#else
        pollfd uart{readiness, POLLOUT, 0};
        if (poll(&uart, 1, kUartPollMs) <= 0 || !(uart.revents & POLLOUT) ||
            (uart.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
        device.SetSpeedCms(command);
        return device.LastWriteOk();
#endif
    }
    bool EmergencyStop() noexcept {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        const bool ok = !fakeMotorFail();
        try { trace("motor_emergency", ok ? "" : "fail"); } catch (...) {}
        return ok;
#else
        return device.EmergencyStop();
#endif
    }
};
struct DriveServo {
#if !defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS
    Servo device;
#endif
    bool open() {
        HardwareCall call(2);
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        return !fakeOpenFail("servo");
#else
        return device.Init() && device.IsReady() && device.SetPWMFreq(kServoPwmHz);
#endif
    }
    bool rawAngle(int a) {
        a = clampHwSteer(a);
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        const bool ok = !fakeOpenFail("servo");
        trace("servo_angle", std::to_string(a) + (ok ? "" : " fail"));
        return ok;
#else
        return device.SetAngle(static_cast<int16_t>(a));
#endif
    }
    bool SetAngle(int a) { HardwareCall call(2); return rawAngle(a); }
};
class Watchdog {
    DriveMotor& motor_;
    DriveServo& servo_;
    const Paths& paths_;
    std::atomic<bool> done_{false};
    std::thread worker_;
    void fail(int kind, int code) noexcept {
        hardwareFault = true;
        try {
            std::thread([code] {
                ::poll(nullptr, 0, kCleanupTimeoutMs + kStopTimeoutMs + 2 * kStopKillMs);
                _exit(code);
            }).detach();
        } catch (...) { motor_.EmergencyStop(); _exit(code); }
        motor_.EmergencyStop();
        // Không ghi I2C chồng lên lời gọi đang treo của chính I2C.
        if (kind != 2) { try { servo_.rawAngle(0); } catch (...) {} }
        warning();
        externalStop(paths_);
        _exit(code);
    }
public:
    std::atomic<int64_t> heartbeat{0}, cleanupAt{0};
    std::atomic<int> cleanupCode{4};
    std::atomic<bool> initialized{false};
    Watchdog(DriveMotor& motor, DriveServo& servo, const Paths& paths)
        : motor_(motor), servo_(servo), paths_(paths) {
        heartbeat = nowMs();
        worker_ = std::thread([this] {
            while (!done_) {
                const auto now = nowMs(), since = hardwareSince.load(), cleanup = cleanupAt.load();
                const int kind = hardwareKind.load();
                const int limit = initialized ? kHardwareTimeoutMs : kInitTimeoutMs;
                if (since && now - since > limit && hardwareSince.load() == since) fail(kind, 4);
                if (cleanup && now - cleanup > kCleanupTimeoutMs) fail(since ? kind : 0, cleanupCode);
                if (!cleanup && !since && initialized && now - heartbeat.load() > kStaleFrameMs)
                    fail(0, 4);
                ::poll(nullptr, 0, kPollMs);
            }
        });
    }
    ~Watchdog() { done_ = true; if (worker_.joinable()) worker_.join(); }
};
struct Command { int64_t ns = 0; float speed = 0; int angle = 0; };
class Control {
    DriveMotor& motor_;
    DriveServo& servo_;
    std::deque<Command> history_{Command{}};
    void remember() {
        history_.push_back({monotonicNs(), speed, angle});
        while (history_.size() > 1 && history_[1].ns < monotonicNs() -
               static_cast<int64_t>(kFrameTimeoutMs) * 1000000) history_.pop_front();
        trace("state", state());
    }
public:
    float speed = 0;
    int angle = 0;
    Control(DriveMotor& motor, DriveServo& servo) : motor_(motor), servo_(servo) {}
    const char* state() const { return speed > 0 ? "CHAY_TIEN" : speed < 0 ? "CHAY_LUI" : "DUNG"; }
    Command at(int64_t ns) const {
        auto result = history_.front();
        for (const auto& c : history_) { if (c.ns > ns) break; result = c; }
        return result;
    }
    bool stop() noexcept {
        speed = 0; angle = 0;
        bool motorOk = false, servoOk = false;
        try { motorOk = motor_.SetSpeedCms(0); } catch (...) {}
        if (!motorOk) motorOk = motor_.EmergencyStop();
        try { servoOk = servo_.SetAngle(0); } catch (...) {}
        try { remember(); } catch (...) {}
        return motorOk && servoOk;
    }
    void key(unsigned char k) {
        if (k >= 'A' && k <= 'Z') k += 'a' - 'A';
        if (k == ' ' || k == 'x') {
            if (!stop()) throw Failure(1, "Không xác nhận được motor 0 rồi servo 0");
        } else if (k == 'w' || k == 's') {
            const float next = clampKeySpeed(speed + (k == 'w' ? kSpeedStepCms : -kSpeedStepCms));
            if (!motor_.SetSpeedCms(next)) throw Failure(1, "Lỗi ghi motor");
            speed = next; remember();
        } else if (k == 'a' || k == 'd' || k == 'c') {
            const int next = k == 'c' ? 0 : clampKeySteer(angle + (k == 'a' ? kSteerStepDeg : -kSteerStepDeg));
            if (!servo_.SetAngle(next)) throw Failure(1, "Lỗi ghi servo");
            angle = next; remember();
        }
    }
};

struct Header { int status = 0, width = 0, height = 0; int64_t ns = 0, wall = 0; };
bool sendAll(int fd, const void* data, size_t size) {
    const auto* p = static_cast<const char*>(data);
    while (size && !stopping) {
        const auto n = ::send(fd, p, size, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        p += n; size -= static_cast<size_t>(n);
    }
    return size == 0;
}
void cameraWorker(int fd) {
    try {
        // Cha tạo con trước preview.start; con chỉ mở camera sau yêu cầu đầu tiên.
        char request;
        if (::recv(fd, &request, 1, 0) != 1) return;
        cv::VideoCapture cap;
        std::string fake;
        bool synthetic = false;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        if (const char* v = std::getenv("UCR_DRIVE_FAKE_CAMERA")) fake = v;
        synthetic = fake == "ok" || fake == "none" || fake == "stall";
        if (fakeOpenFail("camera")) { Header h; h.status = -2; sendAll(fd, &h, sizeof(h)); return; }
#endif
        if (!synthetic && !cap.open(kPipeline, cv::CAP_GSTREAMER)) {
            Header h; h.status = -2; sendAll(fd, &h, sizeof(h)); return;
        }
        unsigned produced = 0;
        while (!stopping && ::recv(fd, &request, 1, 0) == 1) {
            cv::Mat frame;
            if (synthetic) {
                if (fake == "none" || (fake == "stall" && produced >= 3)) {
                    while (!stopping) ::poll(nullptr, 0, kPollMs);
                    break;
                }
                ::poll(nullptr, 0, 2 * kPollMs);
                frame = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(80, 120, 160));
                ++produced;
            } else cap.read(frame);
            Header h; h.ns = monotonicNs(); h.wall = wallMs();
            if (!frame.empty()) {
                if (frame.type() != CV_8UC3 || frame.total() * frame.elemSize() > kMaxFrameBytes)
                    throw std::runtime_error("Khung camera BGR không hợp lệ hoặc quá lớn");
                if (!frame.isContinuous()) frame = frame.clone();
                h.status = 1; h.width = frame.cols; h.height = frame.rows;
            }
            if (!sendAll(fd, &h, sizeof(h))) break;
            if (h.status == 1 && !sendAll(fd, frame.data, frame.total() * frame.elemSize())) break;
            if (!h.status) ::poll(nullptr, 0, kPollMs);
        }
    } catch (...) { Header h; h.status = -2; sendAll(fd, &h, sizeof(h)); }
}
class Camera {
    int fd_ = -1;
    pid_t child_ = -1;
    bool started_ = false;
    bool receive(void* data, size_t size, const std::function<bool()>& tick) {
        auto* p = static_cast<char*>(data);
        while (size) {
            if (!tick()) return false;
            pollfd ready{fd_, POLLIN, 0};
            const int result = poll(&ready, 1, kPollMs);
            if (result < 0 && errno == EINTR) continue;
            if (result < 0) throw Failure(1, "Không chờ được camera");
            if (!result) continue;
            const auto n = ::recv(fd_, p, size, MSG_DONTWAIT);
            if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
            if (n <= 0) throw Failure(1, "Tiến trình camera đã đóng/chết");
            p += n; size -= static_cast<size_t>(n);
        }
        return true;
    }
public:
    explicit Camera(int lockFd) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets) != 0)
            throw Failure(1, "Không tạo được kênh camera");
        child_ = fork();
        if (child_ == 0) {
            ::close(sockets[0]); ::close(lockFd);
            cameraWorker(sockets[1]); ::close(sockets[1]); _exit(0);
        }
        ::close(sockets[1]);
        if (child_ < 0) { ::close(sockets[0]); throw Failure(1, "Không tạo được con camera"); }
        fd_ = sockets[0];
    }
    bool read(cv::Mat& frame, Header& h, const std::function<bool()>& tick) {
        const char request = 1;
        if (!started_) {
            if (::send(fd_, &request, 1, MSG_DONTWAIT | MSG_NOSIGNAL) != 1)
                throw Failure(3, "Không khởi động được camera");
            started_ = true;
        }
        // Mỗi thời điểm chỉ một yêu cầu, không tích lũy khung cũ trong socket.
        if (::send(fd_, &request, 1, MSG_DONTWAIT | MSG_NOSIGNAL) != 1)
            throw Failure(1, "Không yêu cầu được khung camera");
        if (!receive(&h, sizeof(h), tick)) return false;
        if (h.status == -2) throw Failure(3, "Không mở/đọc được camera");
        if (h.status != 1) return true;
        if (h.width <= 0 || h.height <= 0 ||
            static_cast<uint64_t>(h.width) * static_cast<uint64_t>(h.height) * 3 > kMaxFrameBytes)
            throw Failure(1, "Kích thước camera không hợp lệ");
        frame.create(h.height, h.width, CV_8UC3);
        return receive(frame.data, frame.total() * frame.elemSize(), tick);
    }
    ~Camera() {
        if (fd_ >= 0) { ::shutdown(fd_, SHUT_RDWR); ::close(fd_); }
        if (child_ > 0) {
            ::kill(child_, SIGKILL);
            while (waitpid(child_, nullptr, 0) < 0 && errno == EINTR) {}
        }
    }
};

double freeMB(const fs::path& path) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
    if (const char* value = std::getenv("UCR_DRIVE_FAKE_FREE_MB")) {
        size_t used = 0; const double n = std::stod(value, &used);
        if (used != std::strlen(value) || !std::isfinite(n) || n < 0)
            throw std::runtime_error("Dung lượng giả không hợp lệ");
        return n;
    }
#endif
    struct statvfs info{};
    if (statvfs(path.c_str(), &info) != 0) throw std::runtime_error("Không đọc được dung lượng đĩa");
    return static_cast<double>(info.f_bavail) * info.f_frsize / (1024.0 * 1024.0);
}
void makeOutput(Options& o) {
    fs::create_directories(o.outRoot);
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{}; localtime_r(&t, &tm);
    std::ostringstream name; name << std::put_time(&tm, "%Y%m%d_%H%M%S") << '_' << safeLabel(o.label);
    const auto base = o.outRoot / name.str();
    for (uint64_t n = 1; ; ++n) {
        o.out = n == 1 ? base : fs::path(base.string() + '_' + std::to_string(n));
        std::error_code error;
        if (fs::create_directory(o.out, error)) return;
        if (error && error != std::errc::file_exists) throw std::runtime_error(error.message());
    }
}
void writeSession(const Options& o, const std::string& host, const std::string& started,
                  const std::string& ended, int code) {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open((o.out / "session.json.tmp").string());
    file << "{\n\"version\":\"ucr_collect_drive/1\",\n\"git\":" << jsonString(UCR_DRIVE_GIT_REVISION)
         << ",\n\"hostname\":" << jsonString(host) << ",\n\"label\":" << jsonString(o.label)
         << ",\n\"who\":" << jsonString(o.who) << ",\n\"weather\":" << jsonString(o.weather)
         << ",\n\"curtain\":" << jsonString(o.curtain) << ",\n\"lights\":" << jsonString(o.lights)
         << ",\n\"props\":" << jsonString(o.props) << ",\n\"note\":" << jsonString(o.note)
         << ",\n\"started_utc\":" << jsonString(started) << ",\n\"ended_utc\":" << jsonString(ended)
         << ",\n\"pipeline_used\":" << jsonString(kPipeline) << ",\n\"out_root\":" << jsonString(o.outRoot.string())
         << ",\n\"interval_ms\":" << o.interval << ",\n\"reverse_motor\":" << (o.reverseMotor ? "true" : "false")
         << ",\n\"speed_step_cms\":" << kSpeedStepCms << ",\n\"max_speed_cms\":" << kMaxSpeedCms
         << ",\n\"hw_speed_cms\":" << kHwSpeedCms << ",\n\"steer_step_deg\":" << kSteerStepDeg
         << ",\n\"max_steer_deg\":" << kMaxSteerDeg << ",\n\"hw_steer_deg\":" << kHwSteerDeg
         << ",\n\"stale_frame_ms\":" << kStaleFrameMs << ",\n\"exit_code\":" << code << "\n}\n";
    file.close(); fs::rename(o.out / "session.json.tmp", o.out / "session.json");
}
class Writer {
    struct Request { bool on; uint64_t epoch; int64_t wall; };
    struct Frame { cv::Mat image; Header header; Command command; uint64_t epoch = 0, dropped = 0; };
    Options options_;
    std::string host_, started_;
    drive_collect::SessionInfo session_;
    std::mutex mutex_;
    std::condition_variable available_;
    std::deque<Request> requests_;
    Frame pending_;
    bool quit_ = false;
    uint64_t epoch_ = 0, dropped_ = 0;
    std::atomic<bool> recording_{false};
    std::thread worker_;
    static void require(const drive_collect::StoreStatus& status) {
        if (!status) throw std::runtime_error(status.message);
    }
    void run() noexcept {
        try {
            busySince = nowMs();
            drive_collect::PassStore store(options_.outRoot, session_);
            require(store.status());
            if (freeMB(options_.outRoot) < kMinFreeMb) throw std::runtime_error("Đĩa dưới ngưỡng dự phòng");
            writeSession(options_, host_, started_, "", 0);
            pass = store.nextPass(); ready = true;
            uint64_t applied = 0;
            int64_t nextDisk = 0;
            while (true) {
                busySince = 0;
                std::deque<Request> requests;
                Frame frame;
                bool quit;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    available_.wait_for(lock, std::chrono::milliseconds(kPollMs), [&] {
                        return quit_ || !requests_.empty() || !pending_.image.empty();
                    });
                    requests.swap(requests_); frame = std::move(pending_); pending_ = Frame{}; quit = quit_;
                }
                busySince = nowMs();
                if (quit) {
                    if (store.active()) require(store.endPass(finalCode ? "error" : "exit", wallMs()));
                    writeSession(options_, host_, started_, utcNow(), finalCode);
                    break;
                }
                for (const auto& request : requests) {
                    if (request.on) require(store.beginPass(request.wall));
                    else if (store.active()) require(store.endPass("p", request.wall));
                    applied = request.epoch;
                    pass = store.active() ? store.pass() : store.nextPass();
                }
                if (nowMs() >= nextDisk) {
                    if (freeMB(options_.outRoot) < kMinFreeMb) throw std::runtime_error("Đĩa dưới ngưỡng dự phòng");
                    nextDisk = nowMs() + kDiskCheckMs;
                }
                if (frame.image.empty() || !store.active() || !recording_ || frame.epoch != applied) continue;
                cv::Mat gray, lap;
                cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
                cv::Laplacian(gray, lap, CV_64F);
                cv::Scalar mean, deviation; cv::meanStdDev(lap, mean, deviation);
                std::vector<unsigned char> jpeg;
                if (!cv::imencode(".jpg", frame.image, jpeg, {cv::IMWRITE_JPEG_QUALITY, kJpegQuality}))
                    throw std::runtime_error("Không mã hóa được JPEG");
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (quit_ || !recording_ || frame.epoch != epoch_) continue;
                }
                drive_collect::CaptureColumns columns;
                columns.frame = saved.load() + 1; columns.monotonic_ns = frame.header.ns;
                columns.width = frame.image.cols; columns.height = frame.image.rows;
                columns.mean_luma = cv::mean(gray)[0]; columns.lap_var = deviation[0] * deviation[0];
                columns.cmd_speed = frame.command.speed; columns.cmd_steer = frame.command.angle;
                columns.state = frame.command.speed > 0 ? "CHAY_TIEN" : frame.command.speed < 0 ? "CHAY_LUI" : "DUNG";
                columns.dropped_since_last = frame.dropped;
                require(store.saveFrame(jpeg, frame.header.wall, columns));
                ++saved;
            }
        } catch (const std::exception& e) { error = e.what(); failure = 5; }
        catch (...) { failure = 5; }
        busySince = 0; finished = true;
    }
public:
    std::atomic<int> failure{0}, finalCode{0};
    std::atomic<bool> ready{false}, finished{false};
    std::atomic<int64_t> busySince{0};
    std::atomic<uint64_t> saved{0}, pass{1};
    std::string error; // Chỉ đọc sau finished (release/acquire mặc định).
    Writer(const Options& o, const std::string& host, const std::string& started, int argc, char** argv)
        : options_(o), host_(host), started_(started) {
        session_.git = UCR_DRIVE_GIT_REVISION; session_.label = o.label; session_.pipeline = kPipeline;
        session_.argv.assign(argv, argv + argc);
        worker_ = std::thread([this] { run(); });
    }
    void recording(bool on) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++epoch_; recording_ = on;
        requests_.push_back({on, epoch_, wallMs()});
        pending_ = Frame{}; dropped_ = 0;
        available_.notify_one();
    }
    void frame(const cv::Mat& image, const Header& header, const Command& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_.image.empty()) ++dropped_;
        pending_ = {image, header, command, epoch_, dropped_};
        available_.notify_one();
    }
    void stop(int code) {
        std::lock_guard<std::mutex> lock(mutex_);
        finalCode = code; quit_ = true; recording_ = false; available_.notify_one();
    }
    ~Writer() { if (worker_.joinable()) worker_.join(); }
};

int collect(const Options& o, const Paths& paths, int lockFd, const std::string& host, int argc, char** argv) {
    DriveMotor motor(o.reverseMotor);
    DriveServo servo;
    Control control(motor, servo);
    std::unique_ptr<Camera> camera;
    drive::PreviewHost preview;
    std::unique_ptr<Writer> writer;
    std::unique_ptr<Watchdog> watchdog;
    int code = 0;
    bool motorAttempted = false;
    std::string error;
    try {
        camera.reset(new Camera(lockFd));
        if (preview.start() != drive::PreviewStartResult::Started)
            throw Failure(1, "Không chạy được cửa sổ ảnh: " + preview.diagnostic());
        watchdog.reset(new Watchdog(motor, servo, paths));
        trace("state", "DUNG");
        motorAttempted = true;
        if (!motor.init()) throw Failure(1, "Motor::Init thất bại");
        if (!motor.SetSpeedCms(0)) throw Failure(1, "Không xác nhận motor 0 sau Init");
        if (!servo.open()) throw Failure(1, "Không mở được servo/PWM");
        if (!servo.SetAngle(0)) throw Failure(1, "Không trả được servo 0 sau motor 0");
        watchdog->heartbeat = nowMs(); watchdog->initialized = true;
        const auto started = nowMs();
        writer.reset(new Writer(o, host, utcNow(), argc, argv));
        bool running = true, stale = true, haveFrame = false, recording = false;
        int64_t lastFrame = started, nextSave = 0;
        const auto tick = [&]() {
            const auto now = nowMs(); watchdog->heartbeat = now;
            if (stopping) return false;
            if (writer->failure) throw Failure(5, "Đĩa đầy hoặc lỗi ghi dữ liệu");
            const auto busy = writer->busySince.load();
            if (busy && now - busy > kIoTimeoutMs) throw Failure(4, "Luồng ghi dữ liệu quá hạn");
            const auto child = preview.pollChild();
            if (child == drive::PreviewChildStatus::Closed || child == drive::PreviewChildStatus::Died)
                throw Failure(1, "Cửa sổ ảnh đã đóng/chết");
            if (!preview.wasShown() && now - started > kFrameTimeoutMs && haveFrame)
                throw Failure(1, "Cửa sổ ảnh không hiển thị được khung");
            if (now - lastFrame > kStaleFrameMs && !stale) {
                if (!control.stop()) throw Failure(1, "Lỗi dừng khi mất ảnh");
                stale = true;
            }
            if (now - lastFrame > kFrameTimeoutMs)
                throw Failure(haveFrame ? 4 : 3, haveFrame ? "Camera treo" : "Không có ảnh đầu tiên");
            for (unsigned char key : preview.pollKeys()) {
                trace("key", std::string(1, key));
                if (key == 'E' || key == '3') { running = false; break; }
                if (stopping) return false;
                if (key == 'p' || key == 'P') {
                    recording = !recording; writer->recording(recording); nextSave = 0;
                } else if (key == ' ' || key == 'x' || key == 'X' || key == 'c' || key == 'C' ||
                           (!stale && preview.wasShown() && writer->ready)) {
                    control.key(key);
                }
            }
            drive::PreviewState state;
            std::snprintf(state.state, sizeof(state.state), "%s", control.state());
            state.cmd_speed = control.speed; state.cmd_steer = control.angle;
            state.recording = recording; state.pass = writer->pass; state.images = writer->saved;
            state.frame_age_ms = static_cast<double>(now - lastFrame);
            preview.publishState(state);
            return running;
        };
        while (tick()) {
            cv::Mat frame; Header h;
            if (!camera->read(frame, h, tick)) break;
            if (h.status != 1 || frame.empty()) continue;
            if (nowMs() - h.ns / 1000000 > kStaleFrameMs) continue;
            lastFrame = h.ns / 1000000; haveFrame = true; stale = false;
            preview.publishFrame(frame, lastFrame);
            if (recording && nowMs() >= nextSave) {
                writer->frame(frame, h, control.at(h.ns));
                nextSave = nowMs() + o.interval;
            }
        }
    } catch (const Failure& e) { code = e.code; error = e.what(); }
    catch (const std::exception& e) { code = 1; error = e.what(); }
    catch (...) { code = 1; error = "Ngoại lệ không xác định"; }

    if (watchdog) { watchdog->cleanupCode = code ? code : 4; watchdog->cleanupAt = nowMs(); }
    // Dừng trước mọi I/O đóng lượt, join, shutdown preview hay thu hồi camera.
    bool stopped = false;
    if (motorAttempted) stopped = control.stop();
    else {
        const bool motorOk = motor.EmergencyStop();
        bool servoOk = false;
        try { servoOk = servo.SetAngle(0); } catch (...) {}
        stopped = motorOk && servoOk;
    }
    if (!stopped) { if (!code) code = 1; warning(); }
    if (watchdog) watchdog->cleanupCode = code ? code : 4;
    if (writer) writer->stop(code);
    preview.shutdown();
    camera.reset();
    if (writer) {
        const auto deadline = nowMs() + kCleanupTimeoutMs;
        while (!writer->finished && nowMs() < deadline) ::poll(nullptr, 0, kPollMs);
        if (!writer->finished) {
            // Giữ khóa và đối tượng sống tới _exit; không huỷ bộ nhớ của luồng đang kẹt.
            externalStop(paths); warning(); _exit(code ? code : 4);
        }
        if (writer->failure) { code = 5; error = writer->error; }
        writer.reset();
    }
    if (!error.empty()) std::cerr << "Lỗi: " << error << '\n';
    return code;
}
} // namespace

int main(int argc, char** argv) {
    // Như drive_collect: OpenCV một luồng trước mọi fork (preview.cpp dựa vào điều này, tránh TBB treo sau fork).
    cv::setNumThreads(1);
    Options o;
    try { o = parse(argc, argv); }
    catch (const std::exception& e) { std::cerr << "Lỗi tham số: " << e.what() << '\n'; return 2; }
    if (o.help) {
        std::cout << "sudo ./ucr_collect_drive [--label drive] [--out-root THƯ_MỤC] [--interval-ms 1000]\n"
                  << "  [--reverse-motor] [--who NGƯỜI] [--weather THỜI_TIẾT] [--curtain RÈM]\n"
                  << "  [--lights ĐÈN] [--props ĐẠO_CỤ] [--note GHI_CHÚ] [--help]\n"
                  << "W/S +/-5 cm/s; A/D trái/phải 5 độ; C thẳng; Space/X dừng; Esc/Ctrl+C thoát; P ghi.\n";
        return 0;
    }
    struct sigaction action{}; action.sa_handler = signalStop; sigemptyset(&action.sa_mask);
    for (int s : {SIGINT, SIGTERM, SIGHUP}) {
        if (sigaction(s, &action, nullptr) != 0) return 1;
    }
    signal(SIGPIPE, SIG_IGN);
    try {
        const auto paths = executablePaths();
        if (o.outRoot.empty()) o.outRoot = paths.root / "data_raw";
        o.outRoot = fs::absolute(o.outRoot);
        VehicleLock lock;
        std::cout << "============================================================\n"
                  << "KHÔNG CÓ PHANH CHỐNG VẬT CẢN; người giám sát đứng sát xe; Space/X dừng\n"
                  << "============================================================\n";
        if (!externalStop(paths)) { warning(); return 1; }
        int code = 0;
        try {
            if (!stopping) {
                try { makeOutput(o); }
                catch (const std::exception& e) { throw Failure(5, "Không tạo được đầu ra: " + std::string(e.what())); }
                std::array<char, 256> hostname{};
                if (gethostname(hostname.data(), hostname.size() - 1) != 0) throw Failure(1, "Không đọc được tên máy");
                std::cout << "Máy=" << hostname.data() << " git=" << UCR_DRIVE_GIT_REVISION
                          << "\nĐầu ra=" << o.outRoot << " phiên=" << o.out
                          << "\nTốc độ: bước=" << kSpeedStepCms << " trần=" << kMaxSpeedCms << " phần cứng=" << kHwSpeedCms
                          << " cm/s; góc: bước=" << kSteerStepDeg << " trần=" << kMaxSteerDeg << " phần cứng=" << kHwSteerDeg
                          << " độ; đảo motor=" << (o.reverseMotor ? "true" : "false")
                          << "; nhịp ghi=" << o.interval << " ms; mất ảnh=" << kStaleFrameMs << " ms\n"
                          << "Giới hạn ms: poll=" << kPollMs << " UART=" << kUartPollMs << " phần cứng=" << kHardwareTimeoutMs
                          << " Init=" << kInitTimeoutMs << " camera=" << kFrameTimeoutMs << " I/O=" << kIoTimeoutMs
                          << " dọn=" << kCleanupTimeoutMs << " ucr_stop=" << kStopTimeoutMs << '+' << kStopKillMs
                          << "; kiểm đĩa=" << kDiskCheckMs << " dự phòng=" << kMinFreeMb << " MiB; JPEG=" << kJpegQuality
                          << " PWM=" << kServoPwmHz << " Hz; bộ đệm ảnh tối đa=" << kMaxFrameBytes << " byte\n" << std::flush;
                code = collect(o, paths, lock.fd(), hostname.data(), argc, argv);
            }
        } catch (const Failure& e) { code = e.code; std::cerr << e.what() << '\n'; }
        catch (const std::exception& e) { code = 1; std::cerr << e.what() << '\n'; }
        catch (...) { code = 1; }
        if (!externalStop(paths)) { warning(); if (!code) code = 1; }
        return code;
    } catch (const Failure& e) { std::cerr << e.what() << '\n'; return e.code; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    catch (...) { return 1; }
}
