#include "options.h"
#include "pass_store.h"
#ifndef UCR_DRIVE_GIT_REVISION
#define UCR_DRIVE_GIT_REVISION "unknown"
#endif
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#if !defined(UCR_DRIVE_TEST_HOOKS) || !UCR_DRIVE_TEST_HOOKS
#include "hardware/Motor.h"
#include "hardware/Servo.h"
#endif
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <poll.h>
#include <pthread.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

using namespace drive;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
int64_t monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
int64_t wallMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string utcNow() {
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc{}; gmtime_r(&t, &utc);
    std::ostringstream out; out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}
static_assert(std::atomic<int>::is_always_lock_free);
std::atomic<int> stopping{0};
// Chỉ đặt cờ; mọi ghi thiết bị và tài liệu nằm ngoài trình xử lý tín hiệu.
void signalStop(int signal) { if (!stopping.load()) stopping.store(signal); }
std::string signalReason(int signal) {
    if (signal == SIGINT) return "sigint";
    if (signal == SIGHUP) return "sighup";
    if (signal == SIGTSTP) return "sigtstp";
    return "sigterm";
}
struct SteadyClock : IClock { int64_t nowMs() const override { return ::nowMs(); } };
struct Failure : std::runtime_error {
    int code; std::string reason;
    Failure(int c, const std::string& r, const std::string& message) : std::runtime_error(message), code(c), reason(r) {}
};
constexpr const char* busyMessage = "Có thể chương trình khác (test_motor/test_servo/main/ucr_collect_cam) đang dùng xe hoặc camera: đóng chúng trước";
void warning() noexcept {
    const char text[] = "KIEM TRA XE, DUNG CONG TAC NGUON\n";
    pollfd output{2, POLLOUT, 0};
    if (::poll(&output, 1, 0) <= 0 || !(output.revents & POLLOUT)) return;
    const auto ignored = ::write(2, text, sizeof(text) - 1); (void)ignored;
}
constexpr int kMotorInitTimeoutMs = 5000;
constexpr int kCleanupTimeoutMs = 3000;
std::atomic<int> cleanupExitCode{0};
void armCleanupWatchdog(int code) noexcept {
    cleanupExitCode = code;
    try {
        const auto deadline = Clock::now() + std::chrono::milliseconds(kCleanupTimeoutMs);
        std::thread([deadline] {
            std::this_thread::sleep_until(deadline);
            const int code = cleanupExitCode.load();
            _exit(code ? code : 4);
        }).detach();
    } catch (...) { _exit(code ? code : 4); }
}
// Tạo trước các luồng phần cứng; luồng preview tự thừa hưởng mặt nạ riêng trong start().
class InitSignalMask {
    sigset_t original_{};
    bool active_ = true;
public:
    InitSignalMask() {
        sigset_t blocked;
        sigemptyset(&blocked);
        for (int s : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP}) sigaddset(&blocked, s);
        if (pthread_sigmask(SIG_BLOCK, &blocked, &original_) != 0)
            throw Failure(3, "motor_init_fail", "Không chặn được tín hiệu lúc Init");
    }
    void restore() {
        if (active_ && pthread_sigmask(SIG_SETMASK, &original_, nullptr) != 0)
            throw Failure(3, "motor_init_fail", "Không khôi phục được mặt nạ tín hiệu");
        active_ = false;
    }
    ~InitSignalMask() { if (active_) pthread_sigmask(SIG_SETMASK, &original_, nullptr); }
};
// Init sở hữu UART tới khi trả về; hết hạn chỉ thoát, không ghi đè từ luồng khác.
class MotorInitWatchdog {
    std::mutex mutex_;
    std::condition_variable changed_;
    bool done_ = false;
    std::thread worker_;
public:
    MotorInitWatchdog() : worker_([this] {
        std::unique_lock<std::mutex> guard(mutex_);
        if (!changed_.wait_for(guard, std::chrono::milliseconds(kMotorInitTimeoutMs), [this] { return done_; })) {
            warning();
            _exit(3);
        }
    }) {}
    ~MotorInitWatchdog() {
        { std::lock_guard<std::mutex> guard(mutex_); done_ = true; }
        changed_.notify_one();
        worker_.join();
    }
};
}
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
namespace drive {
void hookEvent(const char* event, const std::string& value) {
    static std::mutex traceMutex;
    std::lock_guard<std::mutex> guard(traceMutex);
    if (const char* path = std::getenv("UCR_DRIVE_FAKE_TRACE")) {
        std::ofstream out(path, std::ios::app);
        out << nowMs() << ' ' << event;
        if (!value.empty()) out << ' ' << value;
        out << '\n' << std::flush;
    }
}
}
#endif
namespace {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
struct HookState {
    std::atomic<bool> motorFail{false}, motorHang{false}, servoFail{false}, cameraStall{false};
    std::atomic<bool> jpegFail{false}, csvFail{false}, jpegHang{false}, csvHang{false}, jpegSlow{false};
    std::atomic<int> freeMb{-1};
    std::atomic<int64_t> cameraPauseUntil{0};
} hooks;
bool fakeOpenFail(const char* device) {
    const char* v = std::getenv("UCR_DRIVE_FAKE_OPEN_FAIL");
    return v && std::string(v) == device;
}
void throwTestFault() { throw std::runtime_error("Ngoại lệ thử chưa bắt"); }
void uncaughtTestFault() noexcept { throwTestFault(); }
void pollHooks(size_t& consumed) {
    const char* path = std::getenv("UCR_DRIVE_FAKE_CTRL");
    if (!path) return;
    std::ifstream file(path);
    std::string line;
    size_t index = 0;
    while (std::getline(file, line)) {
        if (file.eof()) break;
        if (index++ < consumed) continue;
        ++consumed;
        if (line == "motor_fail") hooks.motorFail = true;
        else if (line == "motor_hang") hooks.motorHang = true;
        else if (line == "servo_fail") hooks.servoFail = true;
        else if (line == "camera_stall") hooks.cameraStall = true;
        else if (line == "camera_pause_600ms") hooks.cameraPauseUntil = nowMs() + 600;
        else if (line == "jpeg_fail") hooks.jpegFail = true;
        else if (line == "csv_fail") hooks.csvFail = true;
        else if (line == "jpeg_hang") hooks.jpegHang = true;
        else if (line == "csv_hang") hooks.csvHang = true;
        else if (line == "jpeg_slow") hooks.jpegSlow = true;
        else if (line == "throw") uncaughtTestFault();
        else if (line == "stdin_error") throw Failure(4, "stdin_eof", "Tiêm lỗi đọc stdin");
        else if (line == "pause_input_900ms") {
            hookEvent("input_pause", "");
            std::this_thread::sleep_for(900ms);
        }
        else if (line.compare(0, 8, "free_mb ") == 0)
            hooks.freeMb = static_cast<int>(std::min<uint64_t>(number(line.substr(8), "free_mb"), 2147483647));
    }
}
bool failMotor() {
    const char* v = std::getenv("UCR_DRIVE_FAKE_MOTOR_FAIL");
    return hooks.motorFail || (v && std::string(v) == "1");
}
#endif
class VehicleLock {
    int fd_ = -1;
public:
    VehicleLock() {
        const char* path = "/tmp/ucr_vehicle.lock";
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        if (const char* v = std::getenv("UCR_DRIVE_FAKE_LOCK")) path = v;
#endif
        fd_ = ::open(path, O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd_ < 0) throw Failure(3, "open_fail", "Không mở được tệp khóa");
        if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            const int error = errno; ::close(fd_); fd_ = -1;
            if (error == EWOULDBLOCK || error == EAGAIN)
                throw Failure(6, "lock_held", std::string("Đã có tiến trình giữ khóa; ") + busyMessage);
            throw Failure(3, "open_fail", "Không khóa được xe");
        }
    }
    ~VehicleLock() { if (fd_ >= 0) ::close(fd_); }
};
class Terminal : public IKeys {
    termios original_{};
    KeyDecoder decoder_;
public:
    Terminal() {
        if (tcgetattr(0, &original_) != 0) throw Failure(2, "not_tty", "Không đọc được terminal");
        auto raw = original_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_lflag |= ISIG;
        raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &raw) != 0) {
            throw Failure(2, "not_tty", "Không đặt được terminal không-echo");
        }
    }
    ~Terminal() { tcsetattr(0, TCSANOW, &original_); }
    void discard() { tcflush(0, TCIFLUSH); decoder_.discard(); }
    std::vector<int> read(int64_t) override {
        pollfd p{0, POLLIN, 0};
        const int ready = poll(&p, 1, 0);
        if (ready < 0 && errno == EINTR) return {};
        if (ready < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw Failure(4, "stdin_eof", "Terminal đã đóng hoặc lỗi stdin");
        if (ready > 0 && (p.revents & POLLIN)) {
            std::array<unsigned char, 64 * 1024> bytes;
            int available = 0;
            if (ioctl(0, FIONREAD, &available) < 0 || available < 0)
                throw Failure(4, "stdin_eof", "Không đọc được số byte stdin đang chờ");
            // Chốt lượng byte của lượt này; VMIN/VTIME=0 giữ read không chờ byte mới.
            const size_t limit = std::min(bytes.size(), static_cast<size_t>(std::max(available, 1)));
            size_t total = 0;
            const auto deadline = nowMs() + 5;
            int attempts = 0;
            while (total < limit) {
                if (++attempts > 256 || nowMs() >= deadline)
                    throw Failure(4, "stdin_eof", "Đọc tty vượt hạn; dừng an toàn");
                const auto n = ::read(0, bytes.data() + total, limit - total);
                if (n > 0) { total += static_cast<size_t>(n); continue; }
                if (n < 0 && errno == EINTR) continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                throw Failure(4, "stdin_eof", "Hết dữ liệu hoặc lỗi stdin");
            }
            if (total == bytes.size()) {
                discard();
                throw Failure(4, "stdin_eof", "Hàng phím đạt trần 64 KiB; dừng an toàn");
            }
            // Đọc hết lô đang chờ trước; lấy mốc mới sau đọc để không kéo dài hạn tiền tố.
            return decoder_.decode(bytes.data(), total, nowMs());
        }
        return decoder_.decode(nullptr, 0, nowMs());
    }
};
// UART không chặn; giám sát độc lập cả I2C và mọi đường ghi dừng bị treo.
std::atomic<int64_t> hardwareSince{0};
std::atomic<int> hardwareKind{0}; // 1 UART, 2 I2C; không gọi lặp I2C khi bus đang treo.
std::atomic<bool> hardwareFault{false};
struct HardwareCall {
    explicit HardwareCall(int kind = 1) { hardwareKind = kind; hardwareSince = nowMs(); }
    ~HardwareCall() {
        hardwareSince = 0;
        // Watchdog sở hữu đường thoát khi đã chốt; không quay lại cấp lệnh tiến.
        while (hardwareFault) std::this_thread::sleep_for(10ms);
    }
};
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
struct DriveMotor : IMotor {
    bool good = true;
    std::function<void(int64_t)> onEmergencyStop;
    bool init() {
        hookEvent("motor_init", "");
        const char* delay = std::getenv("UCR_DRIVE_FAKE_INIT_MS");
        std::this_thread::sleep_for(std::chrono::milliseconds(delay ? number(delay, "Init giả") : 500));
        bool failed = fakeOpenFail("motor") || fakeOpenFail("motor_init");
        uint64_t failAt = 0;
        if (const char* v = std::getenv("UCR_DRIVE_FAKE_INIT_WRITE_FAIL"))
            failAt = number(v, "Lệnh ghi Init giả");
        // Bảy lần ghi như chuỗi Init; lỗi trước không bị lần ghi sau che mất.
        bool initWriteFailed = false;
        for (uint64_t n = 1; n <= 7; ++n) {
            good = n != failAt;
            if (!good) initWriteFailed = true;
        }
        if (const char* path = std::getenv("UCR_DRIVE_FAKE_CTRL")) {
            std::ifstream file(path);
            std::string line;
            while (std::getline(file, line)) {
                if (!file.eof() && line == "motor_init") failed = true;
            }
        }
        return !failed && !initWriteFailed;
    }
    void SetSpeedCms(float v) override {
        HardwareCall call;
        if (v != 0 && hardwareFault) { good = false; return; }
        good = !failMotor();
        hookEvent("motor_speed", std::to_string(v) + (good ? "" : " fail"));
        if (hooks.motorHang) std::this_thread::sleep_for(2s);
    }
    bool EmergencyStop() noexcept override {
        const bool ok = !failMotor();
        hookEvent("motor_emergency", ok ? "" : "fail");
        if (ok && onEmergencyStop) onEmergencyStop(monotonicNs());
        return ok;
    }
    bool ok() const override { return good; }
};
struct DriveServo : IServo {
    std::function<void(int64_t, int)> onAngle;
    bool open() { return !fakeOpenFail("servo"); }
    bool SetAngle(int v) override {
        HardwareCall call(2);
        hookEvent("servo_angle", std::to_string(v) + (hooks.servoFail ? " fail" : ""));
        if (!hooks.servoFail && onAngle) onAngle(monotonicNs(), v);
        return !hooks.servoFail;
    }
    bool emergencyCenter() {
        hookEvent("servo_angle", hooks.servoFail ? "0 fail" : "0");
        if (!hooks.servoFail && onAngle) onAngle(monotonicNs(), 0);
        return !hooks.servoFail;
    }
};
#else
struct DriveMotor : IMotor {
    Motor device;
    int readiness = -1;
    bool good = false;
    std::function<void(int64_t)> onEmergencyStop;
    bool init() {
        const bool initialized = device.Init();
        readiness = ::open("/dev/ttyTHS1", O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        return initialized && !device.InitHadWriteFailure() && readiness >= 0 && device.LastWriteOk();
    }
    ~DriveMotor() { if (readiness >= 0) ::close(readiness); }
    void SetSpeedCms(float v) override {
        HardwareCall call;
        pollfd uart{readiness, POLLOUT, 0};
        // Không thử lại poll khi EINTR: tín hiệu phải đưa vòng lái sang DỪNG ngay.
        good = poll(&uart, 1, 50) > 0 && (uart.revents & POLLOUT) &&
               !(uart.revents & (POLLERR | POLLHUP | POLLNVAL));
        if (good && !(v != 0 && hardwareFault)) { device.SetSpeedCms(v); good = device.LastWriteOk(); }
        else good = false;
    }
    bool EmergencyStop() noexcept override {
        const bool ok = device.EmergencyStop();
        if (ok && onEmergencyStop) onEmergencyStop(monotonicNs());
        return ok;
    }
    bool ok() const override { return good; }
};
struct DriveServo : IServo {
    Servo device;
    std::function<void(int64_t, int)> onAngle;
    bool open() { HardwareCall call(2); return device.Init() && device.IsReady() && device.SetPWMFreq(50); }
    bool SetAngle(int v) override {
        HardwareCall call(2);
        const bool ok = device.SetAngle(static_cast<int16_t>(v));
        if (ok && onAngle) onAngle(monotonicNs(), v);
        return ok;
    }
    bool emergencyCenter() {
        const bool ok = device.SetAngle(0);
        if (ok && onAngle) onAngle(monotonicNs(), 0);
        return ok;
    }
};
#endif
IMotor* terminateMotor = nullptr;
std::function<void(int64_t, int64_t)> terminateReport;
std::thread::id controlThread;
void emergencyTerminate() noexcept {
    const auto trigger = nowMs();
    hardwareFault = true;
    if (terminateMotor) terminateMotor->EmergencyStop();
    const auto done = nowMs();
    if (terminateMotor) for (int i = 0; i < 3; ++i) {
        std::this_thread::sleep_for(20ms); terminateMotor->EmergencyStop();
    }
    warning();
    // Chỉ đọc Control từ đúng luồng sở hữu; luồng khác vẫn dừng độc lập rồi abort.
    if (std::this_thread::get_id() == controlThread && terminateReport) {
        std::atomic<bool> reported{false};
        try {
            std::thread([&] {
                try { terminateReport(trigger, done); } catch (...) {}
                reported = true;
            }).detach();
            const auto deadline = Clock::now() + 100ms;
            while (!reported && Clock::now() < deadline) std::this_thread::sleep_for(5ms);
        } catch (...) {}
    }
    std::abort();
}
class HardwareWatchdog {
    std::atomic<bool> done_{false};
    std::thread worker_;
public:
    HardwareWatchdog(IMotor& motor, const std::atomic<bool>& initConfirmed,
                     std::function<void(int64_t, int64_t)> report)
        : worker_([this, &motor, &initConfirmed, report] {
        while (!done_) {
            const auto since = hardwareSince.load();
            // Khởi động (Motor::Init, mở servo) có hạn riêng 5 s; 300 ms chỉ áp sau khi đã xác nhận motor 0/servo 0.
            const int64_t limit = initConfirmed ? 300 : kMotorInitTimeoutMs;
            if (since && nowMs() - since > limit && hardwareSince.load() == since) {
                hardwareFault = true;
                // Không gọi servo lần nữa khi chính I2C đang treo.
                motor.EmergencyStop();
                const auto motor0Done = nowMs();
                warning();
                for (int i = 0; i < 3; ++i) { std::this_thread::sleep_for(20ms); motor.EmergencyStop(); }
                // Tổng kết là cố gắng có hạn; đĩa treo không được giữ tiến trình sống.
                std::atomic<bool> reported{false};
                try {
                    std::thread([&] {
                        try { report(since + limit, motor0Done); } catch (...) {}
                        reported = true;
                    }).detach();
                    const auto deadline = Clock::now() + 100ms;
                    while (!reported && Clock::now() < deadline) std::this_thread::sleep_for(5ms);
                } catch (...) {}
                _exit(initConfirmed ? 5 : 3);
            }
            std::this_thread::sleep_for(10ms);
        }
    }) {}
    ~HardwareWatchdog() { done_ = true; worker_.join(); }
};
double freeMB(const fs::path& path) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
    if (hooks.freeMb >= 0) return hooks.freeMb;
    if (const char* v = std::getenv("UCR_DRIVE_FAKE_FREE_MB")) {
        size_t used = 0; const double n = std::stod(v, &used);
        if (used != std::string(v).size() || !std::isfinite(n) || n < 0) throw std::runtime_error("Dung lượng giả không hợp lệ");
        return n;
    }
#endif
    struct statvfs info{};
    if (statvfs(path.c_str(), &info) != 0) throw std::runtime_error("Không đọc được dung lượng ổ đĩa");
    return static_cast<double>(info.f_bavail) * info.f_frsize / (1024.0 * 1024.0);
}
struct Event {
    int width = 0, height = 0;
    uint64_t saved = 0, bytes = 0;
    double free = 0;
};
struct Snapshot {
    int64_t ns = 0;
    float speed = 0;
    int angle = 0;
    State state = State::DUNG;
};
// BT.601 và Laplacian 4 láng giềng, biên phản chiếu; không cần thêm thư viện imgproc.
std::pair<double, double> imageQuality(const cv::Mat& frame) {
    cv::Mat gray(frame.rows, frame.cols, CV_8UC1);
    double sum = 0, lapSum = 0, lapSquared = 0;
    for (int y = 0; y < frame.rows; ++y) {
        const auto* row = frame.ptr<cv::Vec3b>(y);
        auto* target = gray.ptr<unsigned char>(y);
        for (int x = 0; x < frame.cols; ++x) {
            target[x] = static_cast<unsigned char>((row[x][0] * 1868 + row[x][1] * 9617 + row[x][2] * 4899 + 8192) >> 14);
            sum += target[x];
        }
    }
    for (int y = 0; y < frame.rows; ++y) {
        const auto* row = gray.ptr<unsigned char>(y);
        const auto* above = gray.ptr<unsigned char>(y > 0 ? y - 1 : std::min(1, frame.rows - 1));
        const auto* below = gray.ptr<unsigned char>(y + 1 < frame.rows ? y + 1 : std::max(0, y - 1));
        for (int x = 0; x < frame.cols; ++x) {
            const int left = x > 0 ? x - 1 : std::min(1, frame.cols - 1);
            const int right = x + 1 < frame.cols ? x + 1 : std::max(0, x - 1);
            const double lap = above[x] + below[x] + row[left] + row[right] - 4 * row[x];
            lapSum += lap; lapSquared += lap * lap;
        }
    }
    const double pixels = static_cast<double>(frame.total());
    return {sum / pixels, std::max(0.0, lapSquared / pixels - (lapSum / pixels) * (lapSum / pixels))};
}



void makeOutput(Options& o) {
    fs::create_directories(o.outRoot);
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream name; name << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") << '_' << safeLabel(o.label);
    const auto base = o.outRoot / name.str();
    for (uint64_t suffix = 1; ; ++suffix) {
        o.out = suffix == 1 ? base : fs::path(base.string() + '_' + std::to_string(suffix));
        std::error_code error;
        if (fs::create_directory(o.out, error)) return;
        if (!fs::exists(o.out) || !fs::is_directory(o.out))
            throw std::runtime_error("Không tạo được thư mục đầu ra mới: " + error.message());
    }
}
void writeSession(const Options& o, const std::string& started, const std::string& ended,
                  int width = 0, int height = 0, PreviewMonitor monitor = {}) {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open((o.out / "session.json.tmp").string());
    file << "{\n\"label\":" << jsonString(o.label) << ",\n\"started_utc\":" << jsonString(started)
         << ",\n\"ended_utc\":" << jsonString(ended) << ",\n\"pipeline_used\":" << jsonString(o.pipeline)
         << ",\n\"width\":" << width << ",\n\"height\":" << height << ",\n\"argv\":[";
    for (size_t i = 0; i < o.argv.size(); ++i) { if (i) file << ','; file << jsonString(o.argv[i]); }
    std::ostringstream values;
    values << "AUTOSIZE:" << monitor.autosize << ",VISIBLE:" << monitor.visible;
    file << "],\n\"preview_monitor\":" << jsonString(previewMonitorName(monitor.property))
         << ",\n\"preview_monitor_values\":" << jsonString(values.str())
         << ",\n\"preview_monitor_calibrated\":" << (monitor.calibrated ? "true" : "false")
         << ",\n\"version\":\"ucr_drive_collect/1\",\n\"speed_cms\":" << o.speed
         << ",\n\"reverse_speed_cms\":" << o.reverseSpeed
         << ",\"steer_step_deg\":" << o.steerStep << ",\"max_steer_deg\":" << o.maxSteer
         << ",\"hold_timeout_ms\":" << o.holdTimeout << ",\"interval_ms\":" << o.interval
         << ",\"min_free_mb\":" << o.minFree << ",\"jpeg_quality\":90,\"accept_no_obstacle_brake\":true\n}\n";
    file.close(); fs::rename(o.out / "session.json.tmp", o.out / "session.json");
}

// Chỉ luồng ghi mở tệp; hàng đợi chứa đúng một khung, thay khung cũ khi chưa xử lý.
class Capture {
    const Options& o_;
    Control& control_;
    PreviewHost& preview_;
    std::mutex frameMutex_, commandMutex_, statsMutex_;
    std::condition_variable available_;
    cv::Mat pending_;
    Snapshot pendingCommand_, command_;
    int64_t pendingWall_ = 0;
    std::deque<Snapshot> history_{{}};
    Event stats_;
    std::atomic<bool> quit_{false}, cameraDone_{false}, writerDone_{false};
    std::thread camera_, writer_;
    struct RecordingRequest { bool on; std::string reason; int64_t wall, ns; uint64_t epoch; };
    std::mutex recordingMutex_;
    std::deque<RecordingRequest> recordingRequests_;
    std::string endReason_ = "exit";
    std::atomic<uint64_t> recordingEpoch_{0};
    std::atomic<bool> recording_{false};
    std::atomic<uint64_t> pass_{0}, nextPass_{0};
    static void requireStore(const drive_collect::StoreStatus& status) {
        if (!status) throw std::runtime_error(status.message);
    }
    void fail(int status) { int expected = 0; failure.compare_exchange_strong(expected, status); }
    void rememberCommand() {
        history_.push_back(command_);
        while (history_.size() > 1 && history_[1].ns < command_.ns - 6000000000LL) history_.pop_front();
    }
    void cameraLoop() noexcept {
        cv::VideoCapture cap;
        try {
            bool synthetic = false;
            std::string fake;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
            if (const char* v = std::getenv("UCR_DRIVE_FAKE_CAMERA")) fake = v;
            synthetic = fake == "ok" || fake == "none" || fake == "stall";
            if (fakeOpenFail("camera")) { fail(3); cameraDone_ = true; return; }
#endif
            if (!synthetic && !cap.open(o_.pipeline, cv::CAP_GSTREAMER)) { fail(3); cameraDone_ = true; return; }
            uint64_t produced = 0;
            while (!quit_) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                if (hooks.cameraStall || nowMs() < hooks.cameraPauseUntil) { std::this_thread::sleep_for(10ms); continue; }
#endif
                cv::Mat frame;
                if (synthetic) {
                    if (fake == "none" || (fake == "stall" && produced >= 3)) { std::this_thread::sleep_for(10ms); continue; }
                    std::this_thread::sleep_for(20ms);
                    frame = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(80, 120, 160));
                    ++produced;
                } else if (!cap.read(frame) || frame.empty()) { std::this_thread::sleep_for(10ms); continue; }
                const auto ns = monotonicNs();
                // Không nhận ảnh mới sau khi luồng giám sát đã chốt đường dừng độc lập.
                if (hardwareFault) break;
                lastFrameNs.store(ns); // Ngay khi read trả khung, trước mã hoá hay bất kỳ ghi tệp nào.
                preview_.publishFrame(frame, ns / 1000000);
                if (frame.type() != CV_8UC3) throw std::runtime_error("Khung phải là BGR 8 bit, 3 kênh");
                width = frame.cols; height = frame.rows;
                Snapshot atCapture;
                {
                    std::lock_guard<std::mutex> commandGuard(control_.commandMutex);
                    std::lock_guard<std::mutex> guard(commandMutex_);
                    atCapture = history_.front();
                    for (const auto& item : history_) { if (item.ns > ns) break; atCapture = item; }
                }
                atCapture.ns = ns;
                {
                    std::lock_guard<std::mutex> guard(frameMutex_);
                    pending_ = std::move(frame); pendingCommand_ = atCapture; pendingWall_ = wallMs();
                }
                available_.notify_one();
            }
        } catch (...) { fail(4); }
        cap.release();
        cameraDone_ = true;
    }
    void writerLoop() noexcept {
        std::unique_ptr<drive_collect::PassStore> owner;
        try {
            drive_collect::SessionInfo session;
            session.git = UCR_DRIVE_GIT_REVISION; session.argv = o_.argv;
            session.label = o_.label; session.pipeline = o_.pipeline;
            owner.reset(new drive_collect::PassStore(o_.outRoot, session, [this] { writerBeat = nowMs(); }));
            auto& store = *owner;
            requireStore(store.status());
            nextPass_ = store.nextPass();
            int64_t nextSave = 0, lastSaved = 0, passStarted = 0, checkedSpace = 0;
            uint64_t appliedEpoch = 0;
            bool sessionWritten = false, monitorWritten = false;
            uint64_t dropped = 0;
            Event stats;
            while (true) {
                writerBeat = nowMs();
                std::deque<RecordingRequest> requests;
                std::string endReason;
                { std::lock_guard<std::mutex> guard(recordingMutex_);
                  requests.swap(recordingRequests_); endReason = endReason_; }
                for (const auto& request : requests) {
                    if (request.on) {
                        requireStore(store.beginPass(request.wall));
                        passStarted = request.ns;
                    } else if (store.active()) requireStore(store.endPass(request.reason, request.wall));
                    appliedEpoch = request.epoch;
                    pass_ = store.pass(); nextPass_ = store.nextPass();
                }
                if (quit_) {
                    if (store.active()) requireStore(store.endPass(endReason));
                    break;
                }
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                if (hooks.jpegFail || hooks.csvFail) throw std::runtime_error("Tiêm lỗi ghi JPEG/CSV");
                if (hooks.jpegHang || hooks.csvHang) while (true) std::this_thread::sleep_for(100ms);
#endif
                if (nowMs() - checkedSpace >= 100) {
                    stats.free = freeMB(o_.out); checkedSpace = nowMs();
                    if (stats.free < o_.minFree) {
                        fail(2);
                        if (store.active()) store.endPass("disk_low");
                        break;
                    }
                }
                cv::Mat frame;
                Snapshot command;
                int64_t wall = 0;
                {
                    std::unique_lock<std::mutex> guard(frameMutex_);
                    available_.wait_for(guard, 20ms, [&] { return quit_ || !pending_.empty(); });
                    if (quit_) continue;
                    if (pending_.empty()) continue;
                    frame = std::move(pending_); command = pendingCommand_; wall = pendingWall_;
                }
                const auto monitor = preview_.monitor();
                if (!sessionWritten || (monitor.calibrated && !monitorWritten)) {
                    writeSession(o_, o_.started, "", frame.cols, frame.rows, monitor);
                    sessionWritten = true; monitorWritten = monitor.calibrated;
                }
                const bool stopped = command.state == State::DUNG && command.speed == 0;
                if (!store.active() || !recording_ || appliedEpoch != recordingEpoch_ ||
                    command.ns < passStarted || command.ns < nextSave ||
                    (lastSaved && !drive_collect::frameDue(stopped, (command.ns - lastSaved) / 1000000))) {
                    ++dropped; continue;
                }
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                if (hooks.jpegSlow) std::this_thread::sleep_for(1500ms);
#endif
                const auto quality = imageQuality(frame);
                std::vector<unsigned char> encoded;
                if (!cv::imencode(".jpg", frame, encoded, {cv::IMWRITE_JPEG_QUALITY, 90})) throw std::runtime_error("Không mã hóa được JPEG");
                // Bỏ khung cũ nếu P/đường dừng đổi lượt trong lúc mã hóa.
                if (!recording_ || appliedEpoch != recordingEpoch_) continue;
                drive_collect::CaptureColumns columns;
                columns.frame = stats.saved + 1; columns.monotonic_ns = command.ns;
                columns.width = frame.cols; columns.height = frame.rows;
                columns.mean_luma = quality.first; columns.lap_var = quality.second;
                columns.dropped_since_last = dropped; columns.cmd_speed = command.speed;
                columns.cmd_steer = command.angle; columns.state = stateName(command.state);
                requireStore(store.saveFrame(encoded, wall, columns));
                ++stats.saved; stats.bytes += encoded.size();
                { std::lock_guard<std::mutex> guard(statsMutex_); stats_ = stats; }
                lastSaved = command.ns;
                dropped = 0; nextSave = command.ns + static_cast<int64_t>(o_.interval) * 1000000;
                writerBeat = nowMs();
            }
        } catch (...) {
            fail(1); recording_ = false;
            // Lỗi đĩa có thể ngăn cả dòng end; vẫn cố đóng nếu kho còn ghi được.
            if (owner && owner->active()) { try { owner->endPass("error"); } catch (...) {} }
        }
        writerDone_ = true;
    }
public:
    std::atomic<int> failure{0}; // 1 ghi tệp, 2 dung lượng, 3 mở camera, 4 dữ liệu khung.
    std::atomic<int64_t> lastFrameNs{0}, writerBeat{0};
    std::atomic<int> width{0}, height{0};
    Capture(const Options& o, Control& control, PreviewHost& preview) : o_(o), control_(control), preview_(preview) {
        if (o_.recOnStart) requestRecording(true);
        writerBeat = nowMs();
        writer_ = std::thread([this] { writerLoop(); });
        try { camera_ = std::thread([this] { cameraLoop(); }); }
        catch (...) { quit_ = true; writer_.detach(); warning(); _exit(4); }
    }
    void command(int64_t ns, float speed, int angle, State state) {
        std::lock_guard<std::mutex> guard(commandMutex_);
        command_ = {ns, speed, angle, state};
        rememberCommand();
    }
    // Đường dừng độc lập không đọc Control đang kẹt trong lời gọi phần cứng.
    void emergencyCommand(int64_t ns) {
        std::lock_guard<std::mutex> guard(commandMutex_);
        command_.ns = ns; command_.speed = 0; command_.state = State::DUNG;
        rememberCommand();
    }
    void steeringCommand(int64_t ns, int angle) {
        std::lock_guard<std::mutex> guard(commandMutex_);
        command_.ns = ns; command_.angle = angle;
        rememberCommand();
    }
    Event stats() {
        std::lock_guard<std::mutex> guard(statsMutex_);
        auto result = stats_; result.width = width; result.height = height;
        return result;
    }
    bool recording() const { return recording_; }
    uint64_t displayedPass() const { return recording_ ? pass_.load() : nextPass_.load(); }
    void requestRecording(bool on, const std::string& reason = "user") {
        std::lock_guard<std::mutex> guard(recordingMutex_);
        if (recording_ == on) return;
        if (recordingRequests_.size() >= 256) { fail(1); return; }
        const auto epoch = recordingEpoch_.load() + 1;
        recordingRequests_.push_back({on, reason, wallMs(), monotonicNs(), epoch});
        recordingEpoch_ = epoch; recording_ = on;
        available_.notify_one();
    }
    void requestStop(const std::string& reason) {
        { std::lock_guard<std::mutex> guard(recordingMutex_); endReason_ = reason; quit_ = true; }
        recording_ = false; available_.notify_all();
    }
    bool finished() const { return cameraDone_ && writerDone_; }
    void join() { camera_.join(); writer_.join(); }
    // Đường thoát có hạn: luồng đã xong thì join, luồng treo thì detach để _exit không báo rò luồng.
    void reapOrDetach() {
        if (camera_.joinable()) { if (cameraDone_) camera_.join(); else camera_.detach(); }
        if (writer_.joinable()) { if (writerDone_) writer_.join(); else writer_.detach(); }
    }
};
// Luồng nhẹ riêng: lỗi hoặc treo stops.csv/stdout không được giữ vòng điều khiển.
class StopJournal {
    std::mutex mutex_;
    std::deque<StopRecord> pending_;
    std::string diagnostic_;
    std::atomic<bool> quit_{false}, done_{false};
    std::thread worker_;
public:
    explicit StopJournal(const fs::path& out) : worker_([this, out] {
        std::ofstream csv((out / "stops.csv").string());
        csv << "reason,t_trigger_ms,t_motor0_done_ms,latency_ms\n" << std::flush;
        while (true) {
            std::deque<StopRecord> batch;
            std::string diagnostic;
            { std::lock_guard<std::mutex> guard(mutex_); batch.swap(pending_); diagnostic.swap(diagnostic_); }
            if (!diagnostic.empty()) std::cerr << diagnostic << '\n';
            for (const auto& record : batch) {
                csv << record.reason << ',' << record.trigger << ',' << record.done << ',' << record.done - record.trigger << '\n' << std::flush;
                std::cout << "DUNG " << record.reason << ": do tre dung = " << record.done - record.trigger << " ms\n" << std::flush;
            }
            if (quit_) {
                std::lock_guard<std::mutex> guard(mutex_);
                if (pending_.empty() && diagnostic_.empty()) break;
            }
            std::this_thread::sleep_for(10ms);
        }
        done_ = true;
    }) {}
    void push(const StopRecord& record) {
        std::lock_guard<std::mutex> guard(mutex_);
        // Khi đĩa treo, ưu tiên dừng xe và giữ tối đa 100 dòng gần nhất trong summary.
        if (pending_.size() < 4096) pending_.push_back(record);
    }
    void warn(const std::string& diagnostic) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!diagnostic_.empty()) diagnostic_ += '\n';
        diagnostic_ += diagnostic;
    }
    void requestStop() { quit_ = true; }
    bool finished() const { return done_; }
    void join() { worker_.join(); }
    void reapOrDetach() {
        if (!worker_.joinable()) return;
        if (done_) worker_.join(); else worker_.detach();
    }
};
void writeSummary(const Options& o, const Event& e, const Control& c, int code, const std::string& reason,
                  const std::string& started, const std::string& ended) {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open((o.out / "summary.json.tmp").string());
    file << "{\n\"started_utc\":" << jsonString(started) << ",\n\"ended_utc\":" << jsonString(ended)
         << ",\n\"images\":" << e.saved << ",\n\"total_bytes\":" << e.bytes
         << ",\n\"width\":" << e.width << ",\n\"height\":" << e.height
         << ",\n\"exit_code\":" << code << ",\n\"exit_reason\":" << jsonString(reason)
         << ",\n\"emergency_stops\":" << c.emergencyStops << ",\n\"max_speed_cmd\":" << c.maxSpeed
         << ",\n\"max_steer_abs_cmd\":" << c.maxAngle << ",\n\"free_mb_end\":" << e.free << ",\n\"stops\":[";
    const auto count = std::min(c.stopCount, c.stops.size());
    for (size_t i = c.stopCount - count; i < c.stopCount; ++i) {
        const auto& record = c.stops[i % c.stops.size()];
        if (i != c.stopCount - count) file << ',';
        file << "{\"reason\":" << jsonString(record.reason) << ",\"t_trigger_ms\":" << record.trigger
             << ",\"t_motor0_done_ms\":" << record.done << ",\"latency_ms\":" << record.done - record.trigger << '}';
    }
    file << "]\n}\n";
    file.close(); fs::rename(o.out / "summary.json.tmp", o.out / "summary.json");
}
int run(Options o) {
    VehicleLock lock;
    PreviewHost preview(o.preview);
    const auto previewStart = preview.start();
    bool previewHandled = previewStart != PreviewStartResult::Started;
    bool previewUnavailable = previewStart == PreviewStartResult::Unavailable;
    if (previewUnavailable) std::cerr << preview.diagnostic() << '\n';
    if (previewStart != PreviewStartResult::Started)
        std::cerr << "preview_monitor=none values=AUTOSIZE:0,VISIBLE:0 calibrated=false\n";
    o.started = utcNow();
    SteadyClock clock;
    DriveMotor motor;
    DriveServo servo;
    Control control(o, motor, servo, clock);
    bool motorOpened = false, servoOpened = false;
    std::atomic<bool> initConfirmed{false};
    int code = 0;
    std::string reason = "esc", error;
    std::unique_ptr<Terminal> terminal;
    std::unique_ptr<Capture> capture;
    std::unique_ptr<StopJournal> journal;
    InitSignalMask initSignals;
    const auto reportFatal = [&](const char* reason, int code, int64_t trigger, int64_t done) {
        // Luồng điều khiển đang kẹt trong HardwareCall và bị chốt, không sửa Control nữa.
        if (hardwareKind == 1 && servoOpened) servo.emergencyCenter();
        control.recordStop(reason, trigger, done);
        if (capture) capture->requestStop("error");
        if (!o.out.empty()) {
            Event stats;
            if (capture) stats = capture->stats();
            writeSummary(o, stats, control, code, reason, o.started, utcNow());
        }
        if (journal) {
            journal->requestStop();
            while (!journal->finished()) std::this_thread::sleep_for(5ms);
        }
        // Sắp _exit từ luồng giám sát: luồng đã xong thì join, luồng treo thì detach (tránh báo rò luồng).
        if (capture) capture->reapOrDetach();
        if (journal) journal->reapOrDetach();
    };
    HardwareWatchdog watchdog(motor, initConfirmed, [&](int64_t trigger, int64_t done) {
        reportFatal(initConfirmed ? "motor_fail" : "motor_init_fail", initConfirmed ? 5 : 3, trigger, done);
    });
    terminateMotor = &motor;
    controlThread = std::this_thread::get_id();
    terminateReport = [&](int64_t trigger, int64_t done) { reportFatal("exception", 134, trigger, done); };
    struct EmergencyScope {
        IMotor& motor;
        IServo& servo;
        const bool& servoOpened;
        bool active = true;
        ~EmergencyScope() {
            if (active) {
                for (int i = 0; i < 4; ++i) motor.EmergencyStop();
                if (servoOpened) { try { servo.SetAngle(0); } catch (...) {} }
            }
            terminateMotor = nullptr; terminateReport = {};
        }
    } emergencyScope{motor, servo, servoOpened};
    int64_t trigger = nowMs();
    try {
        motorOpened = true;
        {
            MotorInitWatchdog initWatchdog;
            if (!motor.init()) throw Failure(3, "motor_init_fail", std::string("Khởi tạo motor thất bại; ") + busyMessage);
            control.setSpeed(0); // Trước cả servo, camera, tạo thư mục hoặc ghi JSON.
            if (!servo.open()) throw Failure(3, "open_fail", "Không mở được servo");
            servoOpened = true;
            control.setAngle(0);
            initConfirmed = true;
            initSignals.restore(); // Tín hiệu chờ chỉ được giao sau motor 0/servo 0.
        }
        if (const int stopSignal = stopping.load()) {
            trigger = nowMs();
            throw Failure(stopSignal == SIGQUIT ? 143 : 128 + stopSignal,
                          signalReason(stopSignal), "Đã nhận tín hiệu trong Init");
        }
        if (previewUnavailable && o.preview.require_preview)
            throw Failure(7, "preview_unavailable", preview.diagnostic());
        terminal.reset(new Terminal());
        terminal->discard(); // Bỏ phím gõ trước và trong Init, W mới sau đó chạy ngay khi có ảnh.
        preview.discardKeys();
        control.requireFreshStart();
        makeOutput(o);
        writeSession(o, o.started, "");
        journal.reset(new StopJournal(o.out));
        bool monitorLogged = previewStart != PreviewStartResult::Started;
        bool closeLogged = false;
        const auto logMonitor = [&] {
            const auto monitor = preview.monitor();
            std::ostringstream line;
            line << "preview_monitor=" << previewMonitorName(monitor.property)
                 << " values=AUTOSIZE:" << monitor.autosize << ",VISIBLE:" << monitor.visible
                 << " calibrated=" << (monitor.calibrated ? "true" : "false");
            journal->warn(line.str());
            monitorLogged = true;
        };
        control.onStop = [&](const StopRecord& record) {
            journal->push(record);
            if (capture && record.reason == "key_x") capture->requestRecording(false, "stop");
        };
        capture.reset(new Capture(o, control, preview));
        int64_t recordingRejectedUntil = 0;
        control.onRecording = [&] { capture->requestRecording(!capture->recording()); };
        control.onRecordingRejected = [&] { recordingRejectedUntil = nowMs() + 2000; };
        control.onCommand = [&](int64_t ns, float speed, int angle, State state) noexcept {
            try { capture->command(ns, speed, angle, state); } catch (...) { capture->failure = 1; }
        };
        motor.onEmergencyStop = [&](int64_t ns) noexcept {
            try { capture->emergencyCommand(ns); } catch (...) { capture->failure = 1; }
        };
        servo.onAngle = [&](int64_t ns, int angle) noexcept {
            try { capture->steeringCommand(ns, angle); } catch (...) { capture->failure = 1; }
        };
        const auto openedAt = nowMs();
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        size_t consumed = 0;
#endif
        // Luồng hiện tại chỉ điều khiển; camera/JPEG/CSV chạy trên hai luồng riêng.
        while (!control.exiting) {
            trigger = nowMs();
            if (const int stopSignal = stopping.load()) throw Failure(stopSignal == SIGQUIT ? 143 : 128 + stopSignal,
                                        signalReason(stopSignal), "Đã nhận tín hiệu dừng");
            const auto childStatus = preview.pollChild();
            if (!previewHandled && (childStatus == PreviewChildStatus::Closed || childStatus == PreviewChildStatus::Died)) {
                previewHandled = true;
                if (preview.wasShown()) {
                    control.stop(childStatus == PreviewChildStatus::Closed ? "preview_closed" : "preview_died",
                                 trigger, true);
                } else {
                    previewUnavailable = true;
                    if (o.preview.require_preview)
                        throw Failure(7, "preview_unavailable", preview.diagnostic());
                    journal->warn(preview.diagnostic());
                }
            }
            const auto monitor = preview.monitor();
            if (!monitorLogged && (monitor.calibrated || monitor.closed || childStatus == PreviewChildStatus::Died)) logMonitor();
            if (preview.takeWindowWarning()) journal->warn(kPreviewWindowWarning);
            if (monitor.closed && !closeLogged) {
                std::ostringstream line;
                line << "preview: cua so dong (prop=" << previewMonitorName(monitor.property)
                     << " gia tri=" << monitor.close_value << ")";
                journal->warn(line.str());
                closeLogged = true;
            }
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
            pollHooks(consumed);
#endif
            const int status = capture->failure.load();
            if (status) throw Failure(status == 3 ? 3 : 4,
                status == 3 ? "open_fail" : (status == 2 ? "disk_low" : "write_error"),
                status == 3 ? std::string("Không mở được camera; ") + busyMessage : "Lỗi luồng thu/ghi ảnh");
            const auto now = nowMs();
            const auto frameNs = capture->lastFrameNs.load();
            const auto lastFrame = frameNs ? frameNs / 1000000 : openedAt;
            if (now - lastFrame > (frameNs ? 2000 : 5000)) {
                trigger = lastFrame + (frameNs ? 2000 : 5000);
                throw Failure(4, frameNs ? "camera_stall" : "camera_no_frame", "Quá hạn camera");
            }
            if (now - capture->writerBeat.load() > 2000) {
                trigger = capture->writerBeat.load() + 2000;
                throw Failure(4, "camera_stall", "Luồng ghi ảnh mất nhịp sống");
            }
            const bool fresh = frameNs && now - lastFrame <= 500;
            if (!fresh && moving(control.state)) control.stop("frame_stale", lastFrame + 500, true);
            control.tick();
            const bool pendingDiscard = control.discardPending;
            control.discardPending = false;
            const auto readAt = nowMs();
            auto keys = terminal->read(readAt);
            const auto readPreviewKeys = [&]() {
                std::vector<int> result;
                for (unsigned char key : preview.pollKeys()) result.push_back(key == 'E' ? 27 : key == '3' ? 3 : key);
                return result;
            };
            auto previewKeys = previewHandled ? std::vector<int>{} : readPreviewKeys();
            const bool burst = control.keyBurst(keys, previewKeys, readAt);
            keys.insert(keys.end(), previewKeys.begin(), previewKeys.end());
            bool queueDrained = false, queueComplete = true;
            const auto drainPendingKeys = [&]() {
                if (queueDrained) return;
                queueDrained = true; // Một lần mỗi vòng, không quét vô hạn khi phím dừng tới liên tục.
                const bool ttyComplete = drainSafetyKeys(keys, [&]() { return terminal->read(nowMs()); }, nowMs);
                const bool previewComplete = drainSafetyKeys(keys, readPreviewKeys, nowMs);
                queueComplete = ttyComplete && previewComplete;
            };
            if (burst) {
                control.rejectKeyBurst(readAt);
                journal->warn("key_burst: bỏ lô phím chuyển động, cần nhả rồi nhấn mới");
                control.discardPending = false;
            }
            const auto hasKey = [&](int lower) {
                return std::find(keys.begin(), keys.end(), lower) != keys.end() ||
                       std::find(keys.begin(), keys.end(), lower - 'a' + 'A') != keys.end();
            };
            // Lô dồn/đuôi sau dừng đã bỏ mọi phím chạy; chỉ lô còn được xử lý mới cần xét WS/SW.
            const bool conflict = !pendingDiscard && !burst && hasKey('w') && hasKey('s');
            if (conflict) {
                // Nhìn cả lô trước khi gửi motor: WS/SW không được chạy thoáng một hướng.
                control.directionStop("reverse_conflict", readAt);
                control.discardPending = false;
            }
            if (pendingDiscard || burst || conflict) drainPendingKeys();
            bool discardMotion = pendingDiscard || burst || conflict;
            // Giữ vết phân loại lô dồn; chỉ bỏ ngầm đuôi sau phím dừng.
            bool stoppedInBatch = pendingDiscard || conflict;
            for (size_t keyIndex = 0; keyIndex < keys.size(); ++keyIndex) {
                const int key = keys[keyIndex];
                control.noteW(key, readAt); // Tính cả W/S bị bỏ ở đuôi lô cho điều kiện nhận P.
                if (!acceptBatchKey(key, stoppedInBatch)) continue;
                trigger = nowMs();
                // Kiểm lại cờ/fault trước mỗi phím, không cho W cùng lô vượt qua lỗi.
                if (stopping.load() || capture->failure || hardwareFault) break;
                if (key == 3) throw Failure(130, "sigint", "Ctrl+C từ cửa sổ preview");
                if (discardMotion && (motionKey(key) || recordingKey(key))) continue;
                const auto latestFrame = capture->lastFrameNs.load();
                const bool ready = latestFrame && nowMs() - latestFrame / 1000000 <= 500;
                control.key(key, ready, readAt);
                if (control.discardPending || control.exiting) {
                    drainPendingKeys(); control.discardPending = false;
                    discardMotion = true;
                    stoppedInBatch = true;
                    if (control.exiting) break;
                }
            }
            if (!queueComplete && !control.exiting)
                throw Failure(4, "stdin_eof", "Xả hàng phím vượt 256 lượt/64 KiB/5 ms; dừng an toàn");
            PreviewState snapshot;
            std::snprintf(snapshot.state, sizeof(snapshot.state), "%s", stateName(control.state));
            snapshot.cmd_speed = control.speed; snapshot.cmd_steer = control.angle;
            snapshot.images = capture->stats().saved;
            snapshot.recording = capture->recording(); snapshot.pass = capture->displayedPass();
            snapshot.recording_rejected = nowMs() < recordingRejectedUntil;
            preview.publishState(snapshot);
            poll(nullptr, 0, 20);
        }
    } catch (const MotorError& e) { code = initConfirmed ? 5 : 3; reason = initConfirmed ? "motor_fail" : "motor_init_fail"; error = e.what(); }
    catch (const ServoError& e) { code = initConfirmed ? 5 : 3; reason = initConfirmed ? "servo_fail" : "motor_init_fail"; error = e.what(); }
    catch (const Failure& e) { code = e.code; reason = e.reason; error = e.what(); }
    catch (const std::exception& e) { code = initConfirmed ? 4 : 3; reason = initConfirmed ? "write_error" : "motor_init_fail"; error = e.what(); }
    catch (...) { code = initConfirmed ? 4 : 3; reason = initConfirmed ? "write_error" : "motor_init_fail"; error = "Ngoại lệ không xác định"; }
    const bool initFailed = reason == "motor_init_fail";
    control.latched_error = true;
    // Mọi việc chờ/ghi tài liệu sau motor 0 và servo 0.
    try {
        // Lỗi lần motor 0 đầu xảy ra trước mở servo: vẫn thử trả lái sau bốn lần dừng.
        if (motorOpened && !servoOpened && reason == "motor_fail") servoOpened = servo.open();
        if (motorOpened) {
            const bool recorded = control.stopCount &&
                control.stops[(control.stopCount - 1) % control.stops.size()].reason == reason;
            if (initFailed) {
                try {
                    control.setSpeed(0);
                    for (int i = 0; i < 3; ++i) {
                        std::this_thread::sleep_for(20ms); motor.EmergencyStop();
                    }
                } catch (const MotorError&) {} // setSpeed đã thử đủ bốn lần; giữ mã Init 3.
                if (!servoOpened) servoOpened = servo.open();
                if (servoOpened) control.setAngle(0);
            }
            else if (code && !recorded) control.stop(reason, trigger, servoOpened);
            else if (servoOpened) control.halt();
            else control.setSpeed(0);
        } else {
            for (int i = 0; i < 4; ++i) motor.EmergencyStop();
            if (servoOpened) servo.SetAngle(0);
        }
    } catch (const MotorError&) { if (!initFailed) { code = 5; reason = "motor_fail"; } }
    catch (const ServoError&) {
        if (!initFailed && reason != "motor_fail") { code = 5; reason = "servo_fail"; }
        // setAngle đã thử dừng motor; trả lái lần cuối không lặp lại chuỗi dừng đó.
        if (servoOpened) { try { servo.SetAngle(0); } catch (...) {} }
    }
    // Hạn cứng bao trùm cả stderr/JSON, join, terminal và các hàm huỷ khi trả về.
    armCleanupWatchdog(code);
    initSignals.restore();
    if (code == 5 || initFailed) warning();
    if (terminal) terminal->discard();
    preview.discardKeys();
    terminal.reset();
    if (capture) capture->requestStop(code == 130 || code == 143 || code == 129 || code == 148 ? "stop" : code ? "error" : "exit");
    if (journal) journal->requestStop();
    const auto deadline = Clock::now() + 2s;
    while ((capture && !capture->finished()) || (journal && !journal->finished())) {
        if (Clock::now() >= deadline) {
            cleanupExitCode = code ? code : 4;
            const char text[] = "Đã dừng xe; luồng I/O treo, bỏ tổng kết và thoát có hạn.\n";
            const auto ignored = ::write(2, text, sizeof(text)-1); (void)ignored;
            if (capture) capture->reapOrDetach();
            if (journal) journal->reapOrDetach();
            _exit(code ? code : 4);
        }
        std::this_thread::sleep_for(10ms);
    }
    Event latest;
    if (capture) {
        capture->join(); latest = capture->stats();
        if (capture->failure && !code) { code = 4; reason = "write_error"; cleanupExitCode = code; }
    }
    const auto finalMonitor = preview.monitor();
    preview.shutdown(); // Camera đã dừng; motor 0/servo 0 đã gửi, vẫn trong hạn cứng 3 s.
    if (journal) journal->join();
    if (!error.empty()) std::cerr << error << '\n';
    {
        try {
            if (o.out.empty()) makeOutput(o);
            const auto ended = utcNow();
            writeSession(o, o.started, ended, latest.width, latest.height, finalMonitor);
            writeSummary(o, latest, control, code, reason, o.started, ended);
        } catch (const std::exception& e) {
            if (!code) code = 4;
            cleanupExitCode = code;
            std::cerr << "Lỗi ghi tổng kết: " << e.what() << '\n';
        }
    }
    emergencyScope.active = false;
    return code;
}

// Tự kiểm chỉ dùng camera/ổ đĩa/khóa; không tạo bất kỳ đối tượng điều khiển motor/servo nào.
int selftest(const Options& o) {
    int failures = 0, checks = 0;
    const auto report = [&](bool ok, const std::string& item, const std::string& detail) {
        ++checks; if (!ok) ++failures;
        std::cout << (ok ? "OK " : "FAIL ") << item << ": " << detail << std::endl;
    };
    for (const char* name : {"DISPLAY", "XAUTHORITY"}) {
        const char* value = std::getenv(name);
        report(o.preview.no_preview || (value && *value), name,
               value && *value ? value : (o.preview.no_preview ? "bỏ qua vì --no-preview" : "rỗng"));
    }
    bool lockOK = false;
    try { { VehicleLock lock; } lockOK = true; report(true, "lock", "giữ được khóa và nhả ngay"); }
    catch (const Failure& e) { report(false, "lock", e.code == 6 ? "xe đang bận (mã 6)" : e.what()); }
    fs::path temporary;
    bool outputOK = false;
    try {
        fs::create_directories(o.outRoot);
        auto pattern = (o.outRoot / ".ucr_selftest_XXXXXX").string();
        if (!mkdtemp(&pattern[0])) throw std::runtime_error("không tạo được thư mục tạm");
        temporary = pattern;
        std::ofstream probe;
        probe.exceptions(std::ios::badbit | std::ios::failbit);
        probe.open((temporary / "write_probe").string()); probe << "ucr_selftest\n"; probe.close();
        drive_collect::PassStore store(temporary / "pass_store_probe");
        if (!store.status()) throw std::runtime_error(store.status().message);
        outputOK = true;
        report(true, "out_root", fs::absolute(o.outRoot).string());
    } catch (const std::exception& e) { report(false, "out_root", e.what()); }
    try {
        const double free = freeMB(o.outRoot);
        report(free >= o.minFree, "disk", std::to_string(free) + " MiB; cần >= " + std::to_string(o.minFree));
    } catch (const std::exception& e) { report(false, "disk", e.what()); }
    std::array<char, 4096> executable{};
    const auto length = readlink("/proc/self/exe", executable.data(), executable.size()-1);
    const fs::path stop = length > 0 && static_cast<size_t>(length) < executable.size()-1 ?
        fs::path(std::string(executable.data(), static_cast<size_t>(length))).parent_path() / "ucr_stop" : fs::path();
    try {
        report(!stop.empty() && fs::is_regular_file(stop) && access(stop.c_str(), X_OK) == 0,
               "ucr_stop", stop.empty() ? "không xác định được tệp thực thi" : stop.string());
    } catch (const std::exception& e) { report(false, "ucr_stop", e.what()); }

    // VideoCapture::open/read có thể treo trong driver; con riêng cho phép chốt hạn 10 giây.
    const auto waitChild = [](pid_t child, Clock::time_point deadline) {
        if (child < 0) return -1;
        int status = 0;
        while (Clock::now() < deadline && !stopping.load()) {
            const auto result = waitpid(child, &status, WNOHANG);
            if (result == child) return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            if (result < 0 && errno != EINTR) return -1;
            poll(nullptr, 0, 10);
        }
        kill(child, SIGKILL);
        while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
        return -1;
    };
    const auto cameraDeadline = Clock::now() + 10s;
    const pid_t camera = lockOK ? fork() : -1;
    if (camera == 0) {
        int result = 1;
        try {
            cv::VideoCapture capture;
            cv::Mat frame;
            bool synthetic = false;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
            const char* fake = std::getenv("UCR_DRIVE_FAKE_CAMERA");
            synthetic = fake && std::string(fake) == "ok";
            if (fake && std::string(fake) == "none") for (;;) pause();
            if (fakeOpenFail("camera")) _exit(1);
#endif
            if (synthetic || capture.open(o.pipeline, cv::CAP_GSTREAMER)) {
                int frames = 0;
                while (frames < 5 && Clock::now() < cameraDeadline) {
                    if (synthetic) frame = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(80,120,160));
                    else if (!capture.read(frame) || frame.empty()) { poll(nullptr, 0, 10); continue; }
                    ++frames;
                }
                if (frames == 5) {
                    result = 2;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                    if (std::getenv("UCR_DRIVE_FAKE_SELFTEST_JPEG_FAIL")) _exit(2);
#endif
                    if (outputOK && cv::imwrite((temporary / "probe.jpg").string(), frame,
                                               {cv::IMWRITE_JPEG_QUALITY, 90}) &&
                        fs::file_size(temporary / "probe.jpg") > 0) {
                        result = 3;
                        std::ifstream image((temporary / "probe.jpg").string(), std::ios::binary);
                        std::vector<unsigned char> jpeg{std::istreambuf_iterator<char>(image), {}};
                        drive_collect::PassStore store(temporary / "pass_store_frame_probe");
                        drive_collect::CaptureColumns columns;
                        columns.frame = 1; columns.monotonic_ns = monotonicNs();
                        columns.width = frame.cols; columns.height = frame.rows; columns.state = "DUNG";
                        if (store.status() && store.beginPass(wallMs()) &&
                            store.saveFrame(jpeg, wallMs(), columns) && store.endPass("exit")) result = 0;
                    }
                }
            }
        } catch (...) {}
        _exit(result);
    }
    const int cameraResult = waitChild(camera, cameraDeadline);
    report(cameraResult == 0 || cameraResult == 2 || cameraResult == 3, "camera",
           cameraResult == 0 || cameraResult == 2 || cameraResult == 3 ?
           "đã lấy 5 khung trong hạn 10 s" : "không mở/lấy đủ 5 khung trong hạn 10 s");
    report(cameraResult == 0 || cameraResult == 3, "jpeg", cameraResult == 0 || cameraResult == 3 ?
           "đã ghi một JPEG thử" : "không ghi được JPEG thử");
    report(cameraResult == 0, "pass_store", cameraResult == 0 ?
           "đã ghi/đóng lượt thử trong thư mục tạm, không tạo lượt thật" : "không ghi được kho lượt thử");
    try { if (!temporary.empty()) fs::remove_all(temporary); report(true, "cleanup", "đã xóa thư mục thử"); }
    catch (const std::exception& e) { report(false, "cleanup", e.what()); }

    int gitPipe[2];
    std::string revision;
    if (pipe(gitPipe) == 0) {
        const auto source = fs::absolute(fs::path(__FILE__)).parent_path().string();
        const pid_t git = fork();
        if (git == 0) {
            dup2(gitPipe[1], STDOUT_FILENO);
            const int null = open("/dev/null", O_WRONLY);
            if (null >= 0) dup2(null, STDERR_FILENO);
            close(gitPipe[0]); close(gitPipe[1]);
            execlp("git", "git", "-C", source.c_str(), "rev-parse", "--short", "HEAD", static_cast<char*>(nullptr));
            _exit(127);
        }
        close(gitPipe[1]);
        if (waitChild(git, Clock::now() + 1s) == 0) {
            char buffer[128]; const auto count = read(gitPipe[0], buffer, sizeof(buffer));
            if (count > 0) revision.assign(buffer, static_cast<size_t>(count));
            while (!revision.empty() && (revision.back() == '\n' || revision.back() == '\r')) revision.pop_back();
        }
        close(gitPipe[0]);
    }
    if (revision.empty()) std::cout << "WARN git: không rõ" << std::endl;
    else report(true, "git", revision);
    report(!stopping.load(), "signal", stopping.load() ? "đã nhận yêu cầu dừng" : "không có tín hiệu dừng");
    std::cout << (failures ? "FAIL " : "OK ") << "total: " << checks - failures << '/' << checks
              << " mục OK, " << failures << " FAIL" << std::endl;
    return failures ? 1 : 0;
}
}
int main(int argc, char** argv) {
    // Giữ OpenCV một luồng để giảm tải Nano và tránh nhóm luồng TBB.
    cv::setNumThreads(1);
    std::set_terminate(emergencyTerminate);
    struct sigaction action{};
    action.sa_handler = signalStop; sigemptyset(&action.sa_mask);
    for (int s : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP}) sigaction(s, &action, nullptr);
    signal(SIGPIPE, SIG_IGN);
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "ucr_drive_collect --accept-no-obstacle-brake [--out-root data_raw] [--rec-on-start] [--label drive]\n"
                  << "  --pipeline CHUỖI --speed-cms 5 [0..60 cm/s, trần cứng 60] --reverse-speed-cms 3 [>0..5 cm/s]\n"
                  << "  --steer-step-deg 5 --max-steer-deg 40 [0..60]\n"
                  << "  --hold-timeout-ms 800 [700..2000] --interval-ms 1000 --min-free-mb 1024\n"
                  << "  Hạn giữ tối thiểu 700 ms để bao phủ độ trễ tự lặp phím đầu của terminal.\n"
                  << "  --no-preview | --require-preview; --preview-scale 0.5 [0.2..1.0]\n"
                  << "  --preview-fps 10 [1..30]; lỗi preview bắt buộc: mã 7, preview_unavailable\n"
                  << "  --selftest [--out-root DIR] [--pipeline CHUỖI]: tự kiểm không động cơ, không cần tty\n"
                  << "Mở là DUNG (motor 0, góc 0); có ảnh tươi: giữ W để tiến ngay; A trái, D phải; C trả lái.\n"
                  << "S = LÙI (giữ). DỪNG = Space / X / Esc. S KHÔNG CÒN LÀ DỪNG.\n"
                  << "X dừng khẩn rồi DUNG; Esc thoát; Ctrl+C dừng và thoát; P bật/tắt REC khi xe dừng, mặc định REC tắt.\n"
                  << "Đổi hướng: motor 0, nghỉ ít nhất 300 ms, nhả đủ hạn giữ rồi nhấn mới.\n"
                  << "Sau dừng/sự cố: nhả phím đủ --hold-timeout-ms rồi nhấn mới để chạy lại.\n"
                  << "Chạy qua scripts/run_drive.sh để có đường dừng độc lập.\n";
        return 0;
    }
    Options options;
    try {
        options = parse(argc, argv);
        if (!options.selftest && !isatty(0)) throw std::runtime_error("stdin không phải tty (terminal trực tiếp); không chạy qua pipe");
    } catch (const std::exception& e) { std::cerr << "Lỗi tham số: " << e.what() << '\n'; return 2; }
    if (options.selftest) {
        try { return selftest(options); }
        catch (const std::exception& e) { std::cerr << "FAIL total: " << e.what() << '\n'; return 1; }
    }
    std::cerr << "CANH BAO: KHONG CO PHANH ToF; người giám sát phải ở sẵn bên xe.\n"
              << "Mở là DUNG; W tiến; A/D lái; C trả lái; X dừng khẩn rồi DUNG; Esc/Ctrl+C thoát.\n"
              << "S = LÙI (giữ). DỪNG = Space / X / Esc. S KHÔNG CÒN LÀ DỪNG.\n"
              << "Đổi hướng: motor 0, nghỉ ít nhất 300 ms; W+S xung đột thì dừng.\n"
              << "Sau dừng/sự cố: nhả phím đủ --hold-timeout-ms rồi nhấn mới.\n"
              << "Ctrl+C chỉ là một cách dừng. Nếu mất điều khiển: DUNG CONG TAC NGUON motor.\n";
    try { return run(options); }
    catch (const Failure& e) { std::cerr << e.what() << '\n'; return e.code; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 4; }
}
