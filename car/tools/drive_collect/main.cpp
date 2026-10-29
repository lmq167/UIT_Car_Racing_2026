#include "options.h"
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
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sys/file.h>
#include <sys/statvfs.h>
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
volatile std::sig_atomic_t stopping = 0;
// Chỉ đặt cờ; mọi ghi thiết bị và tài liệu nằm ngoài trình xử lý tín hiệu.
void signalStop(int signal) { if (!stopping) stopping = signal; }
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
        if (tcsetattr(0, TCSANOW, &raw) != 0) throw Failure(2, "not_tty", "Không đặt được terminal không-echo");
    }
    ~Terminal() { tcsetattr(0, TCSANOW, &original_); }
    void discard() { tcflush(0, TCIFLUSH); decoder_ = KeyDecoder(); }
    std::vector<int> read(int64_t now) override {
        pollfd p{0, POLLIN, 0};
        const int ready = poll(&p, 1, 0);
        if (ready < 0 && errno == EINTR) return {};
        if (ready < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw Failure(4, "stdin_eof", "Terminal đã đóng hoặc lỗi stdin");
        if (ready > 0 && (p.revents & POLLIN)) {
            unsigned char bytes[64];
            const auto n = ::read(0, bytes, sizeof(bytes));
            if (n < 0 && errno == EINTR) return {};
            if (n <= 0) throw Failure(4, "stdin_eof", "Hết dữ liệu stdin");
            return decoder_.decode(bytes, static_cast<size_t>(n), now);
        }
        return decoder_.decode(nullptr, 0, now);
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
    bool init() {
        hookEvent("motor_init", "");
        const char* delay = std::getenv("UCR_DRIVE_FAKE_INIT_MS");
        std::this_thread::sleep_for(std::chrono::milliseconds(delay ? number(delay, "Init giả") : 500));
        bool failed = fakeOpenFail("motor") || fakeOpenFail("motor_init");
        if (const char* path = std::getenv("UCR_DRIVE_FAKE_CTRL")) {
            std::ifstream file(path);
            std::string line;
            while (std::getline(file, line)) {
                if (!file.eof() && line == "motor_init") failed = true;
            }
        }
        return !failed;
    }
    void SetSpeedCms(float v) override {
        HardwareCall call;
        if (v > 0 && hardwareFault) { good = false; return; }
        good = !failMotor();
        hookEvent("motor_speed", std::to_string(v) + (good ? "" : " fail"));
        if (hooks.motorHang) std::this_thread::sleep_for(2s);
    }
    bool EmergencyStop() noexcept override {
        const bool ok = !failMotor();
        hookEvent("motor_emergency", ok ? "" : "fail");
        return ok;
    }
    bool ok() const override { return good; }
};
struct DriveServo : IServo {
    bool open() { return !fakeOpenFail("servo"); }
    bool SetAngle(int v) override {
        HardwareCall call(2);
        hookEvent("servo_angle", std::to_string(v) + (hooks.servoFail ? " fail" : ""));
        return !hooks.servoFail;
    }
    bool emergencyCenter() { hookEvent("servo_angle", hooks.servoFail ? "0 fail" : "0"); return !hooks.servoFail; }
};
#else
struct DriveMotor : IMotor {
    Motor device;
    int readiness = -1;
    bool good = false;
    bool init() {
        const bool initialized = device.Init();
        readiness = ::open("/dev/ttyTHS1", O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        return initialized && readiness >= 0 && device.LastWriteOk();
    }
    ~DriveMotor() { if (readiness >= 0) ::close(readiness); }
    void SetSpeedCms(float v) override {
        HardwareCall call;
        pollfd uart{readiness, POLLOUT, 0};
        // Không thử lại poll khi EINTR: tín hiệu phải đưa vòng lái sang DỪNG ngay.
        good = poll(&uart, 1, 50) > 0 && (uart.revents & POLLOUT) &&
               !(uart.revents & (POLLERR | POLLHUP | POLLNVAL));
        if (good && !(v > 0 && hardwareFault)) { device.SetSpeedCms(v); good = device.LastWriteOk(); }
        else good = false;
    }
    bool EmergencyStop() noexcept override { return device.EmergencyStop(); }
    bool ok() const override { return good; }
};
struct DriveServo : IServo {
    Servo device;
    bool open() { HardwareCall call(2); return device.Init() && device.IsReady() && device.SetPWMFreq(50); }
    bool SetAngle(int v) override { HardwareCall call(2); return device.SetAngle(static_cast<int16_t>(v)); }
    bool emergencyCenter() { return device.SetAngle(0); }
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
    State state = State::KHOA;
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
void writeSession(const Options& o, const std::string& started, const std::string& ended, int width = 0, int height = 0) {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open((o.out / "session.json.tmp").string());
    file << "{\n\"label\":" << jsonString(o.label) << ",\n\"started_utc\":" << jsonString(started)
         << ",\n\"ended_utc\":" << jsonString(ended) << ",\n\"pipeline_used\":" << jsonString(o.pipeline)
         << ",\n\"width\":" << width << ",\n\"height\":" << height << ",\n\"argv\":[";
    for (size_t i = 0; i < o.argv.size(); ++i) { if (i) file << ','; file << jsonString(o.argv[i]); }
    file << "],\n\"version\":\"ucr_drive_collect/1\",\n\"speed_cms\":" << o.speed
         << ",\"steer_step_deg\":" << o.steerStep << ",\"max_steer_deg\":" << o.maxSteer
         << ",\"hold_timeout_ms\":" << o.holdTimeout << ",\"interval_ms\":" << o.interval
         << ",\"min_free_mb\":" << o.minFree << ",\"jpeg_quality\":90,\"accept_no_obstacle_brake\":true\n}\n";
    file.close(); fs::rename(o.out / "session.json.tmp", o.out / "session.json");
}

// Chỉ luồng ghi mở tệp; hàng đợi chứa đúng một khung, thay khung cũ khi chưa xử lý.
class Capture {
    const Options& o_;
    std::mutex frameMutex_, commandMutex_, statsMutex_;
    std::condition_variable available_;
    cv::Mat pending_;
    Snapshot pendingCommand_, command_;
    int64_t pendingWall_ = 0;
    std::deque<Snapshot> history_;
    Event stats_;
    std::atomic<bool> quit_{false}, cameraDone_{false}, writerDone_{false};
    std::thread camera_, writer_;
    void fail(int status) { int expected = 0; failure.compare_exchange_strong(expected, status); }
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
                lastFrameNs.store(ns); // Ngay khi read trả khung, trước mã hoá hay bất kỳ ghi tệp nào.
                if (frame.type() != CV_8UC3) throw std::runtime_error("Khung phải là BGR 8 bit, 3 kênh");
                width = frame.cols; height = frame.rows;
                Snapshot atCapture;
                {
                    std::lock_guard<std::mutex> guard(commandMutex_);
                    atCapture = command_;
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
        try {
            std::ofstream csv;
            csv.exceptions(std::ios::badbit | std::ios::failbit);
            csv.open((o_.out / "metadata.csv").string());
            csv << "frame,monotonic_ns,wall_time_unix_ms,filename,width,height,mean_luma,lap_var,dropped_since_last,cmd_speed,cmd_steer,state\n" << std::flush;
            int64_t nextSave = 0, checkedSpace = 0;
            bool sessionWritten = false;
            uint64_t dropped = 0;
            Event stats;
            while (!quit_) {
                writerBeat = nowMs();
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                if (hooks.jpegFail || hooks.csvFail) throw std::runtime_error("Tiêm lỗi ghi JPEG/CSV");
                if (hooks.jpegHang || hooks.csvHang) while (true) std::this_thread::sleep_for(100ms);
#endif
                if (nowMs() - checkedSpace >= 100) {
                    stats.free = freeMB(o_.out); checkedSpace = nowMs();
                    if (stats.free < o_.minFree) { fail(2); break; }
                }
                cv::Mat frame;
                Snapshot command;
                int64_t wall = 0;
                {
                    std::unique_lock<std::mutex> guard(frameMutex_);
                    available_.wait_for(guard, 20ms, [&] { return quit_ || !pending_.empty(); });
                    if (quit_) break;
                    if (pending_.empty()) continue;
                    frame = std::move(pending_); command = pendingCommand_; wall = pendingWall_;
                }
                if (!sessionWritten) {
                    writeSession(o_, o_.started, "", frame.cols, frame.rows); sessionWritten = true;
                }
                if (command.ns < nextSave) { ++dropped; continue; }
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                if (hooks.jpegSlow) std::this_thread::sleep_for(1500ms);
#endif
                const auto quality = imageQuality(frame);
                std::ostringstream filename;
                filename << "frame_" << std::setw(8) << std::setfill('0') << stats.saved + 1 << ".jpg";
                const auto target = o_.out / filename.str();
                const fs::path temp = target.string() + ".tmp";
                std::vector<unsigned char> encoded;
                if (!cv::imencode(".jpg", frame, encoded, {cv::IMWRITE_JPEG_QUALITY, 90})) throw std::runtime_error("Không mã hóa được JPEG");
                std::ofstream image;
                image.exceptions(std::ios::badbit | std::ios::failbit);
                image.open(temp.string(), std::ios::binary);
                image.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
                image.close(); fs::rename(temp, target);
                ++stats.saved; stats.bytes += encoded.size();
                csv << std::setprecision(12) << stats.saved << ',' << command.ns << ',' << wall << ',' << filename.str()
                    << ',' << frame.cols << ',' << frame.rows << ',' << quality.first << ',' << quality.second << ',' << dropped
                    << ',' << command.speed << ',' << command.angle << ',' << stateName(command.state) << '\n' << std::flush;
                { std::lock_guard<std::mutex> guard(statsMutex_); stats_ = stats; }
                dropped = 0; nextSave = command.ns + static_cast<int64_t>(o_.interval) * 1000000;
                writerBeat = nowMs();
            }
        } catch (...) { fail(1); }
        writerDone_ = true;
    }
public:
    std::atomic<int> failure{0}; // 1 ghi tệp, 2 dung lượng, 3 mở camera, 4 dữ liệu khung.
    std::atomic<int64_t> lastFrameNs{0}, writerBeat{0};
    std::atomic<int> width{0}, height{0};
    explicit Capture(const Options& o) : o_(o) {
        writerBeat = nowMs();
        writer_ = std::thread([this] { writerLoop(); });
        try { camera_ = std::thread([this] { cameraLoop(); }); }
        catch (...) { quit_ = true; writer_.detach(); warning(); _exit(4); }
    }
    void command(const Control& c) {
        std::lock_guard<std::mutex> guard(commandMutex_);
        command_ = {monotonicNs(), c.speed, c.angle, c.state};
        history_.push_back(command_);
        while (history_.size() > 1 && history_[1].ns < command_.ns - 6000000000LL) history_.pop_front();
    }
    Event stats() {
        std::lock_guard<std::mutex> guard(statsMutex_);
        auto result = stats_; result.width = width; result.height = height;
        return result;
    }
    void requestStop() { quit_ = true; available_.notify_all(); }
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
    std::atomic<bool> quit_{false}, done_{false};
    std::thread worker_;
public:
    explicit StopJournal(const fs::path& out) : worker_([this, out] {
        std::ofstream csv((out / "stops.csv").string());
        csv << "reason,t_trigger_ms,t_motor0_done_ms,latency_ms\n" << std::flush;
        while (true) {
            std::deque<StopRecord> batch;
            { std::lock_guard<std::mutex> guard(mutex_); batch.swap(pending_); }
            for (const auto& record : batch) {
                csv << record.reason << ',' << record.trigger << ',' << record.done << ',' << record.done - record.trigger << '\n' << std::flush;
                std::cout << "DUNG " << record.reason << ": do tre dung = " << record.done - record.trigger << " ms\n" << std::flush;
            }
            if (quit_) {
                std::lock_guard<std::mutex> guard(mutex_);
                if (pending_.empty()) break;
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
    const auto reportFatal = [&](const char* reason, int code, int64_t trigger, int64_t done) {
        // Luồng điều khiển đang kẹt trong HardwareCall và bị chốt, không sửa Control nữa.
        if (hardwareKind == 1 && servoOpened) servo.emergencyCenter();
        control.recordStop(reason, trigger, done);
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
        }
        control.setSpeed(0); // Trước cả servo, camera, tạo thư mục hoặc ghi JSON.
        if (!servo.open()) throw Failure(3, "open_fail", "Không mở được servo");
        servoOpened = true;
        control.setAngle(0);
        initConfirmed = true;
        if (stopping) {
            trigger = nowMs();
            throw Failure(stopping == SIGQUIT ? 143 : 128 + stopping,
                          signalReason(stopping), "Đã nhận tín hiệu trong Init");
        }
        terminal.reset(new Terminal());
        terminal->discard(); // Phím gõ trong Init không được mở khóa sau khởi tạo.
        makeOutput(o);
        writeSession(o, o.started, "");
        journal.reset(new StopJournal(o.out));
        control.onStop = [&](const StopRecord& record) { journal->push(record); };
        capture.reset(new Capture(o));
        const auto openedAt = nowMs();
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        size_t consumed = 0;
#endif
        // Luồng hiện tại chỉ điều khiển; camera/JPEG/CSV chạy trên hai luồng riêng.
        while (!control.exiting) {
            trigger = nowMs();
            if (stopping) throw Failure(stopping == SIGQUIT ? 143 : 128 + stopping,
                                        signalReason(stopping), "Đã nhận tín hiệu dừng");
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
            if (!fresh && control.state == State::CHAY) control.stop("frame_stale", lastFrame + 500);
            control.tick();
            if (control.discardPending) { terminal->discard(); control.discardPending = false; }
            const auto keys = terminal->read(nowMs());
            for (int key : keys) {
                trigger = nowMs();
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
                hookEvent("key", key == 27 ? "ESC" : std::string(1, static_cast<char>(key)));
#endif
                // Kiểm lại cờ/fault trước mỗi phím, không cho W cùng lô vượt qua lỗi.
                if (stopping || capture->failure || hardwareFault) break;
                const auto latestFrame = capture->lastFrameNs.load();
                const bool ready = latestFrame && nowMs() - latestFrame / 1000000 <= 500;
                control.key(key, ready);
                capture->command(control);
                if (control.discardPending || control.exiting || nowMs() - now >= 20) {
                    terminal->discard(); control.discardPending = false; break;
                }
            }
            capture->command(control);
            poll(nullptr, 0, 20);
        }
    } catch (const MotorError& e) { code = initConfirmed ? 5 : 3; reason = initConfirmed ? "motor_fail" : "motor_init_fail"; error = e.what(); }
    catch (const ServoError& e) { code = initConfirmed ? 5 : 3; reason = initConfirmed ? "servo_fail" : "motor_init_fail"; error = e.what(); }
    catch (const Failure& e) { code = e.code; reason = e.reason; error = e.what(); }
    catch (const std::exception& e) { code = initConfirmed ? 4 : 3; reason = initConfirmed ? "write_error" : "motor_init_fail"; error = e.what(); }
    catch (...) { code = initConfirmed ? 4 : 3; reason = initConfirmed ? "write_error" : "motor_init_fail"; error = "Ngoại lệ không xác định"; }
    const bool initFailed = reason == "motor_init_fail";
    control.latched_error = true;
    const bool warned = code == 5 || initFailed;
    if (warned) warning();
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
            else if (code && !recorded) control.stop(reason, trigger, servoOpened, true);
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
    if ((code == 5 || initFailed) && !warned) warning();
    terminal.reset();
    if (capture) capture->requestStop();
    if (journal) journal->requestStop();
    const auto deadline = Clock::now() + 2s;
    while ((capture && !capture->finished()) || (journal && !journal->finished())) {
        if (Clock::now() >= deadline) {
            const char text[] = "Đã dừng xe; luồng I/O treo, bỏ tổng kết và thoát có hạn.\n";
            const auto ignored = ::write(2, text, sizeof(text)-1); (void)ignored;
            if (capture) capture->reapOrDetach();
            if (journal) journal->reapOrDetach();
            _exit(code ? code : 4);
        }
        std::this_thread::sleep_for(10ms);
    }
    Event latest;
    if (capture) { capture->join(); latest = capture->stats(); }
    if (journal) journal->join();
    if (!error.empty()) std::cerr << error << '\n';
    {
        try {
            if (o.out.empty()) makeOutput(o);
            const auto ended = utcNow();
            writeSession(o, o.started, ended, latest.width, latest.height);
            writeSummary(o, latest, control, code, reason, o.started, ended);
        } catch (const std::exception& e) { std::cerr << "Lỗi ghi tổng kết: " << e.what() << '\n'; if (!code) code = 4; }
    }
    emergencyScope.active = false;
    return code;
}
}
int main(int argc, char** argv) {
    std::set_terminate(emergencyTerminate);
    struct sigaction action{};
    action.sa_handler = signalStop; sigemptyset(&action.sa_mask);
    for (int s : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP}) sigaction(s, &action, nullptr);
    signal(SIGPIPE, SIG_IGN);
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "ucr_drive_collect --accept-no-obstacle-brake [--out-root runs_drive] [--label drive]\n"
                  << "  --pipeline CHUỖI --speed-cms 5 --steer-step-deg 5 --max-steer-deg 10\n"
                  << "  --hold-timeout-ms 800 [100..2000] --interval-ms 1000 --min-free-mb 1024\n"
                  << "R rồi Y trong 5 giây sau khung đầu; giữ W để tiến; A trái, D phải; C trả lái.\n"
                  << "S/Space dừng, X dừng khẩn và khóa lại, Esc thoát; ảnh luôn được ghi.\n"
                  << "Chạy qua scripts/run_drive.sh để có đường dừng độc lập.\n";
        return 0;
    }
    Options options;
    try {
        options = parse(argc, argv);
        if (!isatty(0)) throw std::runtime_error("stdin không phải tty (terminal trực tiếp); không chạy qua pipe");
    } catch (const std::exception& e) { std::cerr << "Lỗi tham số: " << e.what() << '\n'; return 2; }
    std::cerr << "CANH BAO: KHONG CO PHANH ToF; người giám sát phải ở sẵn bên xe.\n"
              << "Ctrl+C chỉ là một cách dừng. Nếu mất điều khiển: DUNG CONG TAC NGUON motor.\n";
    try { return run(options); }
    catch (const Failure& e) { std::cerr << e.what() << '\n'; return e.code; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 4; }
}
