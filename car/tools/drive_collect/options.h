#ifndef UCR_DRIVE_COLLECT_OPTIONS_H
#define UCR_DRIVE_COLLECT_OPTIONS_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <array>
#include <functional>
#include <exception>
#include <ctime>
#if defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 8
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace drive {
constexpr const char* defaultPipeline =
    "nvarguscamerasrc sensor-mode=5 wbmode=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1";

struct Options {
    fs::path outRoot = "runs_drive", out;
    std::string pipeline = defaultPipeline, label = "drive";
    std::string started;
    std::vector<std::string> argv;
    double speed = 5;
    int steerStep = 5, maxSteer = 10;
    uint64_t holdTimeout = 800, interval = 1000, minFree = 1024;
    bool accepted = false;
};

inline uint64_t number(const std::string& text, const std::string& name) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Cần số nguyên không âm: " + name);
    try { return std::stoull(text); }
    catch (...) { throw std::runtime_error("Số quá lớn: " + name); }
}

inline std::string jsonString(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
        else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
        else out << static_cast<char>(c);
    }
    out << '"';
    return out.str();
}

inline std::string safeLabel(const std::string& label) {
    std::string result;
    for (unsigned char c : label) {
        if ((c & 0xc0) == 0x80) continue;
        result += ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                   (c >= '0' && c <= '9') || c == '_' || c == '-') ? static_cast<char>(c) : '_';
    }
    return result.empty() ? "drive" : result;
}

inline Options parse(int argc, char** argv) {
    Options o;
    for (int i = 0; i < argc; ++i) o.argv.emplace_back(argv[i]);
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--accept-no-obstacle-brake") { o.accepted = true; continue; }
        if (key != "--out-root" && key != "--label" && key != "--pipeline" &&
            key != "--speed-cms" && key != "--steer-step-deg" && key != "--max-steer-deg" &&
            key != "--hold-timeout-ms" && key != "--interval-ms" && key != "--min-free-mb")
            throw std::runtime_error("Tham số không biết: " + key);
        if (++i == argc || std::string(argv[i]).compare(0, 2, "--") == 0)
            throw std::runtime_error("Thiếu giá trị tham số: " + key);
        const std::string value = argv[i];
        if (value.empty()) throw std::runtime_error("Giá trị rỗng: " + key);
        if (key == "--out-root") o.outRoot = value;
        else if (key == "--label") o.label = value;
        else if (key == "--pipeline") {
            if (value.find_first_not_of(" \t\r\n") == std::string::npos)
                throw std::runtime_error("Pipeline (chuỗi xử lý camera) rỗng");
            o.pipeline = value;
        } else if (key == "--speed-cms") {
            size_t used = 0;
            try { o.speed = std::stod(value, &used); }
            catch (...) { throw std::runtime_error("--speed-cms phải là số hữu hạn từ 0 đến 5"); }
            if (used != value.size() || !std::isfinite(o.speed) || o.speed < 0 || o.speed > 5)
                throw std::runtime_error("--speed-cms phải là số hữu hạn từ 0 đến 5 (trần cứng)");
        } else {
            const auto n = number(value, key);
            if (n > static_cast<uint64_t>(std::numeric_limits<int>::max()))
                throw std::runtime_error("Giá trị quá lớn: " + key);
            if (key == "--steer-step-deg" || key == "--max-steer-deg") {
                if ((key == "--steer-step-deg" && n < 1) || (key == "--max-steer-deg" && n > 10))
                    throw std::runtime_error(key + " cần bước dương và trần góc từ 0 đến 10 độ");
                if (key == "--steer-step-deg") o.steerStep = static_cast<int>(n);
                else o.maxSteer = static_cast<int>(n);
            } else if (key == "--hold-timeout-ms") {
                if (n < 100 || n > 2000) throw std::runtime_error(key + " phải trong [100, 2000]");
                o.holdTimeout = n;
            } else if (key == "--interval-ms") {
                if (!n) throw std::runtime_error(key + " phải lớn hơn 0");
                o.interval = n;
            } else o.minFree = n;
        }
    }
    if (!o.accepted) throw std::runtime_error("Thiếu --accept-no-obstacle-brake: công cụ không có phanh ToF");
    return o;
}

struct IMotor {
    virtual ~IMotor() = default;
    virtual void SetSpeedCms(float speed) = 0;
    virtual bool EmergencyStop() noexcept = 0;
    virtual bool ok() const = 0;
};
struct IServo {
    virtual ~IServo() = default;
    virtual bool SetAngle(int angle) = 0;
};
struct IClock {
    virtual ~IClock() = default;
    virtual int64_t nowMs() const = 0;
};
struct IKeys {
    virtual ~IKeys() = default;
    virtual std::vector<int> read(int64_t now) = 0;
};
// Phân biệt Esc đơn với chuỗi CSI/SS3 của terminal, kể cả khi byte tới rời nhau.
class KeyDecoder {
    enum class Escape { None, Pending, Sequence } escape_ = Escape::None;
    int64_t escapeAt_ = 0;
public:
    std::vector<int> decode(const unsigned char* bytes, size_t size, int64_t now) {
        std::vector<int> keys;
        if (escape_ != Escape::None && now - escapeAt_ >= 50) {
            if (escape_ == Escape::Pending) keys.push_back(27);
            escape_ = Escape::None;
        }
        for (size_t i = 0; i < size; ++i) {
            const int b = bytes[i];
            if (escape_ == Escape::Pending) {
                escape_ = (b == '[' || b == 'O') ? Escape::Sequence : Escape::None;
                escapeAt_ = now;
            } else if (escape_ == Escape::Sequence) {
                if (b >= 0x40 && b <= 0x7e) escape_ = Escape::None;
                escapeAt_ = now;
            } else if (b == 27) { escape_ = Escape::Pending; escapeAt_ = now; }
            else if (b >= 32 && b < 127) keys.push_back(b);
        }
        return keys;
    }
};
enum class State { KHOA, SAN_SANG, CHAY };
inline const char* stateName(State s) {
    return s == State::KHOA ? "KHOA" : (s == State::SAN_SANG ? "SAN_SANG" : "CHAY");
}
struct MotorError : std::runtime_error { MotorError() : std::runtime_error("Ghi motor thất bại") {} };
struct ServoError : std::runtime_error { ServoError() : std::runtime_error("Ghi servo thất bại") {} };
struct StopRecord {
    std::string reason;
    int64_t trigger = 0, done = 0;
};
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
void hookEvent(const char* event, const std::string& value);
#endif

class Control {
    const Options& o_;
    IMotor& motor_;
    IServo& servo_;
    IClock& clock_;
    bool confirming_ = false;
    int64_t confirmAt_ = 0, lastW_ = 0, motor0Done_ = 0;
    float completedSpeed_ = 0;
    int completedAngle_ = 0;
    static int64_t commandNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void publishCommand(int64_t ns) {
        const auto appliedState = latched_error ? State::KHOA :
            (completedSpeed_ > 0 ? State::CHAY : (state == State::CHAY ? State::SAN_SANG : state));
        if (onCommand) onCommand(ns, completedSpeed_, completedAngle_, appliedState);
    }
    bool writeSpeed(float v) {
        std::lock_guard<std::mutex> guard(commandMutex);
        motor_.SetSpeedCms(v);
        const auto ns = commandNs();
        if (v == 0) motor0Done_ = clock_.nowMs();
        if (!motor_.ok()) return false;
        completedSpeed_ = v;
        publishCommand(ns);
        return true;
    }
    void emergencyStop() {
        std::lock_guard<std::mutex> guard(commandMutex);
        const bool ok = motor_.EmergencyStop();
        const auto ns = commandNs();
        if (ok) { completedSpeed_ = 0; publishCommand(ns); }
    }
    void transition(State next) {
        std::lock_guard<std::mutex> guard(commandMutex);
        if (state == next) return;
        state = next;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        hookEvent("state", stateName(state));
#endif
        // CHAY chỉ có hiệu lực khi lệnh tiến đã hoàn tất.
        if (next != State::CHAY) publishCommand(commandNs());
    }
public:
    State state = State::KHOA;
    float speed = 0, maxSpeed = 0;
    int angle = 0, maxAngle = 0;
    uint64_t emergencyStops = 0;
    bool exiting = false;
    bool latched_error = false, discardPending = false;
    std::array<StopRecord, 100> stops;
    size_t stopCount = 0;
    // Camera chờ công bố lệnh đang ghi rồi tra lịch sử theo mốc chụp trước đó.
    std::mutex commandMutex;
    std::function<void(int64_t, float, int, State)> onCommand;
    std::function<void(const StopRecord&)> onStop;
    Control(const Options& o, IMotor& m, IServo& s, IClock& c) : o_(o), motor_(m), servo_(s), clock_(c) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        hookEvent("state", "KHOA");
#endif
    }
    void setSpeed(float v) {
        if (v > 0 && (state != State::CHAY || latched_error)) return;
        const auto trigger = clock_.nowMs();
        const bool written = writeSpeed(v);
        speed = v;
        if (!written) {
            latched_error = true;
            speed = 0;
            if (v != 0) writeSpeed(0);
            for (int i = 0; i < 3; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                emergencyStop();
            }
            if (!stopCount || stops[(stopCount - 1) % stops.size()].reason != "motor_fail")
                recordStop("motor_fail", trigger, motor0Done_);
            throw MotorError();
        }
        maxSpeed = std::max(maxSpeed, v);
    }
    void setAngle(int v) {
        bool written;
        {
            std::lock_guard<std::mutex> guard(commandMutex);
            written = servo_.SetAngle(v);
            const auto ns = commandNs();
            if (written) { completedAngle_ = v; publishCommand(ns); }
        }
        if (!written) {
            latched_error = true;
            const auto trigger = clock_.nowMs();
            setSpeed(0);
            if (!stopCount || stops[(stopCount - 1) % stops.size()].reason != "servo_fail")
                recordStop("servo_fail", trigger, motor0Done_);
            for (int i = 0; i < 3; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                emergencyStop();
            }
            throw ServoError();
        }
        angle = v;
        maxAngle = std::max(maxAngle, std::abs(v));
    }
    void recordStop(const std::string& reason, int64_t trigger, int64_t done) {
        StopRecord record{reason, trigger, done};
        stops[stopCount++ % stops.size()] = record;
        if (onStop) onStop(record);
    }
    // Dừng motor trước cả khi trả lái hoặc xử lý lỗi ghi motor.
    void halt() {
        confirming_ = false;
        try { setSpeed(0); }
        catch (...) { try { setAngle(0); } catch (...) {} transition(State::KHOA); throw; }
        setAngle(0);
        transition(State::KHOA);
    }
    void stop(const std::string& reason, int64_t trigger, bool center = false, bool lock = false) {
        confirming_ = false;
        discardPending = true;
        std::exception_ptr failure;
        try { setSpeed(0); } catch (...) { failure = std::current_exception(); }
        if (reason != "motor_fail" || !stopCount || stops[(stopCount - 1) % stops.size()].reason != reason)
            recordStop(reason, trigger, motor0Done_);
        transition(lock ? State::KHOA : (state == State::KHOA ? State::KHOA : State::SAN_SANG));
        if (center) {
            try { setAngle(0); } catch (...) { if (!failure) failure = std::current_exception(); }
        }
        if (failure) std::rethrow_exception(failure);
    }
    int64_t deadmanAt() const { return lastW_ + static_cast<int64_t>(o_.holdTimeout); }
    void tick() {
        const auto now = clock_.nowMs();
        if (confirming_ && now - confirmAt_ > 5000) confirming_ = false;
        if (state == State::CHAY && now - lastW_ >= static_cast<int64_t>(o_.holdTimeout)) {
            stop("deadman", deadmanAt());
        }
    }
    void key(int k, bool cameraReady = true, int64_t readAt = -1) {
        const auto stopsBeforeTick = stopCount;
        tick();
        if (stopCount != stopsBeforeTick) return;
        if (k >= 'A' && k <= 'Z') k += 'a' - 'A';
        if (k != 27 && (k > 127 || std::string("rywads cx").find(static_cast<char>(k)) == std::string::npos)) return;
        if (k == 27) { stop("key_esc", clock_.nowMs(), true, true); exiting = true; return; }
        if (k == 'x') { ++emergencyStops; stop("key_x", clock_.nowMs(), true, true); return; }
        if (k == 's' || k == ' ') {
            stop(k == 's' ? "key_s" : "key_space", clock_.nowMs()); return;
        }
        if (latched_error) return;
        if (state == State::KHOA) {
            if (latched_error || !cameraReady) { confirming_ = false; return; }
            if (confirming_ && k == 'y') { confirming_ = false; transition(State::SAN_SANG); }
            else if (!confirming_ && k == 'r') { confirming_ = true; confirmAt_ = clock_.nowMs(); }
            else confirming_ = false;
            return;
        }
        if (k == 'w' && cameraReady) {
            const auto received = readAt >= 0 ? readAt : clock_.nowMs();
            if (clock_.nowMs() - received >= static_cast<int64_t>(o_.holdTimeout)) return;
            // Cả lô phím dùng chung mốc đọc; W tồn không được gia hạn theo lúc xử lý.
            if (state == State::CHAY && lastW_ == received) return;
            transition(State::CHAY); setSpeed(static_cast<float>(o_.speed)); lastW_ = received;
        } else if (k == 'a') setAngle(static_cast<int>(std::min<int64_t>(o_.maxSteer, static_cast<int64_t>(angle) + o_.steerStep)));
        else if (k == 'd') setAngle(static_cast<int>(std::max<int64_t>(-o_.maxSteer, static_cast<int64_t>(angle) - o_.steerStep)));
        else if (k == 'c') setAngle(0);
    }
};
}
#endif
