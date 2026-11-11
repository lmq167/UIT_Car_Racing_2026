#pragma once
#include "options.h"
#include "hooks.h"
#include <functional>
#include <sys/stat.h>
#include "../collect_common/preview.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
#include "hardware/Motor.h"
#include "hardware/Servo.h"
#include "sensor/BNO055.h"
#if (!defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS) && !defined(UCR_AUTO_NO_TOF)
#include "sensor/TOF.h"
#endif
#include "common/ConfigValue.h"
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
#include <limits>
#include <mutex>
#include <exception>
#include <poll.h>
#include <pthread.h>
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
using namespace auto_drive;
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
struct SignalStop {}; // Tín hiệu tới trong lúc Init: thoát mã 0 qua đường dừng chung.
// Chép từ công cụ thu cũ: chặn tín hiệu khi Motor::Init/mở servo để usleep/ghi UART không bị cắt ngang.
// Tạo trước luồng watchdog để luồng đó thừa hưởng mặt nạ; tín hiệu chờ sẽ tới luồng chính sau restore().
class InitSignalMask {
    sigset_t original_{};
    bool active_ = true;
public:
    InitSignalMask() {
        sigset_t blocked;
        sigemptyset(&blocked);
        for (int s : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP}) sigaddset(&blocked, s);
        if (pthread_sigmask(SIG_BLOCK, &blocked, &original_) != 0)
            throw Failure(1, "Không chặn được tín hiệu lúc Init");
    }
    void restore() {
        if (active_ && pthread_sigmask(SIG_SETMASK, &original_, nullptr) != 0)
            throw Failure(1, "Không khôi phục được mặt nạ tín hiệu");
        active_ = false;
    }
    ~InitSignalMask() { if (active_) pthread_sigmask(SIG_SETMASK, &original_, nullptr); }
};
void warning() noexcept {
    const char text[] = "KIEM TRA XE, DUNG CONG TAC NGUON\n";
    pollfd out{STDERR_FILENO, POLLOUT, 0};
    if (poll(&out, 1, 0) > 0 && (out.revents & POLLOUT)) {
        const auto ignored = ::write(STDERR_FILENO, text, sizeof(text) - 1); (void)ignored;
    }
}
void trace(const char* event, const std::string& value = {}) {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
    // Lấy giờ và ghi dưới cùng một khóa để các luồng không ghi vết đảo thứ tự giờ.
    static std::mutex traceMutex;
    std::lock_guard<std::mutex> traceLock(traceMutex);
    if (const char* path = std::getenv("UCR_AUTO_FAKE_TRACE")) {
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
class VehicleLock {
    int fd_ = -1;
public:
    VehicleLock() {
        const char* path = "/tmp/ucr_vehicle.lock";
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        if (const char* value = std::getenv("UCR_AUTO_FAKE_LOCK")) path = value;
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
struct Paths { std::string root, stop; };
Paths executablePaths() {
    std::array<char, 4096> buf{};
    const auto n = readlink("/proc/self/exe", buf.data(), buf.size() - 1);
    if (n <= 0 || static_cast<size_t>(n) >= buf.size() - 1)
        throw Failure(1, "Không tìm được đường dẫn thật của tệp chạy");
    const std::string executable(buf.data(), static_cast<size_t>(n));
    const auto dir = parentPath(executable);
    auto root = dir;
    while (!regularFile(root + "/config.yaml")
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
           && !regularFile(root + "/control.yaml")
#endif
    ) {
        const auto parent = parentPath(root);
        if (parent == root) throw Failure(1, "Không tìm thấy config.yaml");
        root = parent;
    }
    const auto stop = dir + "/ucr_stop";
    if (!regularFile(stop) || access(stop.c_str(), X_OK) != 0)
        throw Failure(1, "Thiếu ucr_stop cạnh tệp chạy");
    return {root, stop};
}
// posix_spawn dùng được sau khi preview/ghi ảnh đã có luồng; không chạy shell.
bool externalStop(const Paths& paths) noexcept {
    try {
        const auto binary = paths.stop, config = paths.root + "/config.yaml";
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
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
    Motor device;
    int readiness = -1;
    ~DriveMotor() { if (readiness >= 0) ::close(readiness); }
#endif
    explicit DriveMotor(bool r) : reverse(r) {}
    bool init() {
        HardwareCall call(3);
        trace("motor_init");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
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
        const float command = applyMotorSign(clampAutoSpeed(v), reverse);
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
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
    bool GetPositionCm(float* cm) {
        HardwareCall call(1);
        if (hardwareFault) return false;
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        *cm=static_cast<float>(hooks.odoCm); return hooks.encoderOk;
#else
        return device.GetPositionCm(cm);
#endif
    }
    bool EmergencyStop() noexcept {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        const bool ok = !fakeMotorFail();
        try { trace("motor_emergency", ok ? "" : "fail"); } catch (...) {}
        return ok;
#else
        return device.EmergencyStop();
#endif
    }
};
struct SensorSample {
    double yaw_deg=std::numeric_limits<double>::quiet_NaN(), odo_cm=std::numeric_limits<double>::quiet_NaN();
    bool sensor_ok=false;
    int64_t sensor_ms=-1;
    uint64_t fault_id=0;
};
struct DriveImu {
    bool initialized=false;
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
    std::unique_ptr<BNO055> device;
    int axis=2;
    double sign=1, bias=0, yaw=0;
    Clock::time_point previous;
    double Axis(const BNO055::Vector3& v) const { return axis==0 ? v.x : axis==1 ? v.y : v.z; }
#endif
    void init(const Paths& paths) {
        trace("imu_init");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        (void)paths;
        { HardwareCall call(2); if (fakeOpenFail("imu")) throw Failure(1,"Lỗi khởi tạo IMU"); }
        trace("imu_drift");
        { HardwareCall call(2); if (fakeOpenFail("imu_drift")) throw Failure(1,"Lỗi đo trôi IMU"); }
#else
        cv::FileStorage config(paths.root+"/config.yaml",cv::FileStorage::READ);
        if (!config.isOpened()) throw Failure(1,"Không đọc được config.yaml cho IMU");
        std::string calibration; int drift;
        common::ReadConfigValue("imu_calibration_file",config["imu_calibration_file"],calibration);
        common::ReadConfigValue("imu_drift_ms",config["imu_drift_ms"],drift);
        common::ReadConfigValue("yaw_axis",config["yaw_axis"],axis);
        common::ReadConfigValue("yaw_sign",config["yaw_sign"],sign);
        if (drift>=4000) throw Failure(1,"imu_drift_ms phải nhỏ hơn 4000 ms");
        if (calibration.empty() || drift<1 || axis<0 || axis>2 || (sign!=1 && sign!=-1))
            throw Failure(1,"Cấu hình IMU không hợp lệ");
        device.reset(new BNO055("/dev/i2c-1",calibration));
        { HardwareCall call(2); if (!device->initialize()) throw Failure(1,"Lỗi khởi tạo IMU"); }
        trace("imu_drift");
        { HardwareCall call(2); if (!device->measureYawRateDrift(std::chrono::milliseconds(drift)))
            throw Failure(1,"Lỗi đo trôi IMU"); }
        bias=device->yawRateBiasDegPerSec();
        if (axis!=2) {
            double sum=0; int count=0;
            const auto end=Clock::now()+std::chrono::milliseconds(drift);
            do {
                BNO055::Data data;
                { HardwareCall call(2); if (!device->read(data)) throw Failure(1,"Lỗi đo trôi trục IMU"); }
                sum+=Axis(data.gyroscopeDegPerSec); ++count;
                ::poll(nullptr,0,10);
            } while (Clock::now()<end);
            bias=sum/count;
        }
        if (!std::isfinite(bias)) throw Failure(1,"Độ trôi IMU không hữu hạn");
        previous=Clock::now();
#endif
        initialized=true;
    }
    bool read(SensorSample& sample) {
        if (!initialized) return false;
        HardwareCall call(2);
        if (hardwareFault) return false;
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        sample.yaw_deg=hooks.yawDeg; sample.sensor_ms=hooks.sensorMs;
        return hooks.imuOk;
#else
        BNO055::Data data;
        sample.sensor_ms=nowMs();
        const bool ok=device->read(data);
        const auto now=Clock::now();
        const double dt=std::chrono::duration<double>(now-previous).count(); previous=now;
        if (!ok) return false;
        yaw+=(Axis(data.gyroscopeDegPerSec)-bias)*dt*sign;
        sample.yaw_deg=yaw; return std::isfinite(yaw);
#endif
    }
};
// ToF 4 cảm biến hai bên (Quân duyệt 11/11): CHỈ ĐỌC và in khi xe đứng (trước G / sau Space) để xác nhận ánh xạ khe -> bánh.
// Không đọc khi xe đang chạy nên không ảnh hưởng điều khiển; mọi lỗi ToF chỉ in cảnh báo, không dừng chương trình.
#if (!defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS) && !defined(UCR_AUTO_NO_TOF)
#define UCR_AUTO_TOF_REAL 1
#else
#define UCR_AUTO_TOF_REAL 0
#endif
struct DriveTof {
    bool initialized=false;
#if UCR_AUTO_TOF_REAL
    sensor::TOF device;
    int64_t lastPrintMs=0;
#endif
    void init(const DriveImu& imu) {
#if UCR_AUTO_TOF_REAL
        // ToF cấp địa chỉ qua 0x29: IMU ở 0x29 sẽ xung đột (BTC cấm), nên bỏ ToF.
        if (imu.initialized && imu.device && imu.device->address()==0x29) {
            std::cerr<<"TOF BO QUA: IMU o 0x29 trung dia chi khoi tao ToF\n"; return;
        }
        // Init mất vài giây tới chục giây và ghi I2C/GPIO của chính các ToF (mã BTC); xe đang đứng, chưa có G.
        std::cout<<"TOF: dang khoi tao (mat vai giay den chuc giay, Ctrl-C bi hoan)..."<<std::endl;
        if (!device.Init("/dev/i2c-1")) { std::cerr<<"TOF LOI: "<<device.LastError()<<'\n'; return; }
        initialized=true;
        std::cout<<"TOF: khoi tao OK (doc va in khi xe dung)\n"<<std::flush;
#else
        (void)imu;
#endif
    }
    void pollAndPrint() {
#if UCR_AUTO_TOF_REAL
        if (!initialized || hardwareFault) return;
        const int64_t now=nowMs();
        if (now-lastPrintMs<500) return;
        lastPrintMs=now;
        sensor::TOFReadings r;
        bool ok;
        try { HardwareCall call(2); ok=device.Read(r,std::chrono::milliseconds(500)); }
        catch (const std::exception& e) { std::cerr<<"TOF LOI doc: "<<e.what()<<'\n'; return; }
        std::cout<<"TOF mm(rangeStatus):";
        for (std::size_t i=0;i<r.size();++i) {
            std::cout<<" ["<<i<<"] ";
            if (!r[i].initialized) std::cout<<"khong_khoi_tao";
            else if (r[i].valid) std::cout<<r[i].distanceMm<<'('<<int(r[i].rangeStatus)<<')';
            else std::cout<<"x("<<int(r[i].rangeStatus)<<",api"<<r[i].apiError<<(r[i].timedOut?",cu":"")<<')';
        }
        std::cout<<(ok ? "" : " LOI_API")<<'\n'<<std::flush;
#endif
    }
};
struct DriveServo {
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
    Servo device;
#endif
    bool open() {
        HardwareCall call(2);
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        return !fakeOpenFail("servo");
#else
        return device.Init() && device.IsReady() && device.SetPWMFreq(kServoPwmHz);
#endif
    }
    bool rawAngle(int a) {
        a = clampAutoSteer(a);
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        const bool ok = !fakeOpenFail("servo");
        trace("servo_angle", std::to_string(a) + (ok ? "" : " fail"));
        return ok;
#else
        return device.SetAngle(static_cast<int16_t>(a));
#endif
    }
    bool SetAngle(int a) {
        HardwareCall call(2);
        if (hardwareFault) return false; // Như motor: watchdog đã nhận quyền dừng.
        return rawAngle(a);
    }
};
// Đường dự phòng khi terminate: dừng motor trước servo/helper, giữ khóa tới khi thoát.
DriveMotor* terminateMotor = nullptr;
DriveServo* terminateServo = nullptr;
const Paths* terminatePaths = nullptr;
void emergencyTerminate() noexcept {
    hardwareFault = true;
    try {
        std::thread([] {
            ::poll(nullptr, 0, kCleanupTimeoutMs + kStopTimeoutMs + 2 * kStopKillMs);
            _exit(4);
        }).detach();
    } catch (...) {}
    if (terminateMotor) {
        terminateMotor->EmergencyStop();
        for (int i = 0; i < 3; ++i) { ::poll(nullptr, 0, 20); terminateMotor->EmergencyStop(); }
    }
    if (terminateServo && hardwareKind != 2) { try { terminateServo->rawAngle(0); } catch (...) {} }
    if (terminatePaths) externalStop(*terminatePaths);
    warning();
    std::abort();
}
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
        // Như công cụ thu cũ: lặp lệnh dừng 3 lần cách 20 ms phòng lần đầu bị nuốt.
        for (int i = 0; i < 3; ++i) { ::poll(nullptr, 0, 20); motor_.EmergencyStop(); }
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
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        fake = "ok";
        if (const char* v = std::getenv("UCR_AUTO_FAKE_CAMERA")) fake = v;
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
        if (::send(fd_, &request, 1, MSG_DONTWAIT | MSG_NOSIGNAL) != 1) {
            try {
                if (receive(&h, sizeof(h), tick) && h.status == -2)
                    throw Failure(3, "Không mở/đọc được camera");
            } catch (const Failure& e) {
                if (e.code != 1) throw;
            }
            throw Failure(1, "Không yêu cầu được khung camera");
        }
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


} // namespace
