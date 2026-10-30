#ifndef UCR_DRIVE_COLLECT_OPTIONS_H
#define UCR_DRIVE_COLLECT_OPTIONS_H

#include "preview.h"

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
    PreviewOptions preview;
    fs::path outRoot = "runs_drive", out;
    std::string pipeline = defaultPipeline, label = "drive";
    std::string started;
    std::vector<std::string> argv;
    double speed = 5;
    int steerStep = 5, maxSteer = 10;
    uint64_t holdTimeout = 800, interval = 1000, minFree = 1024;
    bool accepted = false, selftest = false;
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
        std::string error;
        const auto preview = parsePreviewOption(argc, argv, i, o.preview, error);
        if (preview == PreviewOptionResult::Error) throw std::runtime_error(error);
        if (preview == PreviewOptionResult::Parsed) continue;
        const std::string key = argv[i];
        if (key == "--selftest") { o.selftest = true; continue; }
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
                if (n < 700 || n > 2000) throw std::runtime_error(key + " phải trong [700, 2000]");
                o.holdTimeout = n;
            } else if (key == "--interval-ms") {
                if (!n) throw std::runtime_error(key + " phải lớn hơn 0");
                o.interval = n;
            } else o.minFree = n;
        }
    }
    if (!o.accepted && !o.selftest) throw std::runtime_error("Thiếu --accept-no-obstacle-brake: công cụ không có phanh ToF");
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
    // Xả lô phím không được hủy Esc đơn đang chờ đủ 50 ms.
    void discard() { if (escape_ == Escape::Sequence) escape_ = Escape::None; }
    std::vector<int> decode(const unsigned char* bytes, size_t size, int64_t now) {
        std::vector<int> keys;
        // Chỉ bỏ CSI/SS3 cụt khi lượt đọc rỗng; đuôi đang chờ phải được nuốt trước.
        if (escape_ == Escape::Sequence && size == 0 && now - escapeAt_ >= 50)
            escape_ = Escape::None;
        // Đọc tiền tố đang chờ trước khi chốt Esc đơn.
        if (escape_ == Escape::Pending && now - escapeAt_ >= 50 &&
            (size == 0 || (bytes[0] != '[' && bytes[0] != 'O'))) {
            keys.push_back(27);
            escape_ = Escape::None;
        }
        for (size_t i = 0; i < size; ++i) {
            const int b = bytes[i];
            if (b == 3) keys.push_back(b);
            else if (b == 27) { escape_ = Escape::Pending; escapeAt_ = now; }
            else if (escape_ == Escape::Pending) {
                // Phím giữ W có thể chen giữa Esc và phần còn lại của mũi tên.
                if (b == '[' || b == 'O') { escape_ = Escape::Sequence; escapeAt_ = now; }
                else if (b == 'x' || b == 'X' || b == 's' || b == 'S' || b == ' ') keys.push_back(b);
            } else if (escape_ == Escape::Sequence) {
                if (b >= 0x40 && b <= 0x7e) escape_ = Escape::None;
                escapeAt_ = now;
            } else if (b >= 32 && b < 127) keys.push_back(b);
        }
        return keys;
    }
};
constexpr int kBurstKeyCount = 4;
inline bool motionKey(int key) {
    if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
    return key == 'w' || key == 'a' || key == 'd' || key == 'c';
}
inline bool safetyKey(int key) {
    return key == 27 || key == 3 || key == 'x' || key == 'X' ||
           key == 's' || key == 'S' || key == ' ';
}
// Đọc không chờ: giữ phím an toàn, bỏ chuyển động; quá trần thì bên gọi phải thoát.
template <class Reader, class Clock>
bool drainSafetyKeys(std::vector<int>& retained, Reader read, Clock now) {
    const auto started = now();
    size_t total = 0;
    for (int attempt = 0; attempt < 256 && now() - started < 5; ++attempt) {
        const auto keys = read();
        if (keys.empty()) return true;
        total += keys.size();
        for (int key : keys) if (safetyKey(key)) retained.push_back(key);
        if (total >= 65536) return false;
    }
    return false;
}
class KeyBurstGuard {
    uint64_t hold_;
    int64_t lastRead_;
public:
    KeyBurstGuard(uint64_t hold, int64_t started) : hold_(hold), lastRead_(started) {}
    bool reject(const std::vector<int>& tty, const std::vector<int>& preview, int64_t readAt) {
        if (tty.empty() && preview.empty()) return false;
        const bool quiet = readAt - lastRead_ >= static_cast<int64_t>((hold_+1)/2);
        lastRead_ = readAt;
        return quiet && (std::count_if(tty.begin(), tty.end(), motionKey) >= kBurstKeyCount ||
                         std::count_if(preview.begin(), preview.end(), motionKey) >= kBurstKeyCount);
    }
};

enum class State { DUNG, CHAY };
inline const char* stateName(State s) {
    return s == State::DUNG ? "DUNG" : "CHAY";
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
inline bool acceptBatchKey(int key, bool stoppedInBatch) {
    // Đuôi chuyển động sau phím dừng phải bị bỏ trước cả ghi vết.
    if (stoppedInBatch && motionKey(key)) return false;
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
    hookEvent("key", key == 27 ? "ESC" : std::string(1, static_cast<char>(key)));
#endif
    return true;
}

class Control {
    const Options& o_;
    IMotor& motor_;
    IServo& servo_;
    IClock& clock_;
    KeyBurstGuard burstGuard_;
    bool awaitingPress_ = false;
    int64_t lastHeld_ = 0, lastW_ = 0, motor0Done_ = 0;
    void requireFreshPress() {
        awaitingPress_ = true;
        lastHeld_ = clock_.nowMs();
    }
    float completedSpeed_ = 0;
    int completedAngle_ = 0;
    static int64_t commandNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void publishCommand(int64_t ns) {
        const auto appliedState = !latched_error && completedSpeed_ > 0 ? State::CHAY : State::DUNG;
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
    State state = State::DUNG;
    float speed = 0, maxSpeed = 0;
    int angle = 0, maxAngle = 0;
    uint64_t emergencyStops = 0;
    bool exiting = false;
    bool latched_error = false, discardPending = false;
    // Gọi sau khi xả phím lúc khởi động: W đang giữ từ trước không được tính là lần nhấn mới.
    void requireFreshStart() { requireFreshPress(); }
    bool keyBurst(const std::vector<int>& tty, const std::vector<int>& preview, int64_t readAt) {
        return burstGuard_.reject(tty, preview, readAt);
    }
    void rejectKeyBurst(int64_t readAt) {
        requireFreshPress();
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        hookEvent("key_burst", "");
#endif
        if (state == State::CHAY) stop("key_burst", readAt, true);
    }
    std::array<StopRecord, 100> stops;
    size_t stopCount = 0;
    // Camera chờ công bố lệnh đang ghi rồi tra lịch sử theo mốc chụp trước đó.
    std::mutex commandMutex;
    std::function<void(int64_t, float, int, State)> onCommand;
    std::function<void(const StopRecord&)> onStop;
    Control(const Options& o, IMotor& m, IServo& s, IClock& c)
        : o_(o), motor_(m), servo_(s), clock_(c), burstGuard_(o.holdTimeout, c.nowMs()) {
#if defined(UCR_DRIVE_TEST_HOOKS) && UCR_DRIVE_TEST_HOOKS
        hookEvent("state", "DUNG");
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
            requireFreshPress();
            transition(State::DUNG);
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
            requireFreshPress();
            transition(State::DUNG);
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
        requireFreshPress();
        try { setSpeed(0); }
        catch (...) { try { setAngle(0); } catch (...) {} transition(State::DUNG); throw; }
        setAngle(0);
        transition(State::DUNG);
    }
    void stop(const std::string& reason, int64_t trigger, bool center = false) {
        requireFreshPress();
        discardPending = true;
        std::exception_ptr failure;
        try { setSpeed(0); } catch (...) { failure = std::current_exception(); }
        if (reason != "motor_fail" || !stopCount || stops[(stopCount - 1) % stops.size()].reason != reason)
            recordStop(reason, trigger, motor0Done_);
        transition(State::DUNG);
        if (center) {
            try { setAngle(0); } catch (...) { if (!failure) failure = std::current_exception(); }
        }
        if (failure) std::rethrow_exception(failure);
    }
    int64_t deadmanAt() const { return lastW_ + static_cast<int64_t>(o_.holdTimeout); }
    void tick() {
        const auto now = clock_.nowMs();
        if (state == State::CHAY && now - lastW_ >= static_cast<int64_t>(o_.holdTimeout)) {
            stop("deadman", deadmanAt());
        }
    }
    void key(int k, bool cameraReady = true, int64_t readAt = -1) {
        const auto stopsBeforeTick = stopCount;
        tick();
        if (k >= 'A' && k <= 'Z') k += 'a' - 'A';
        if (k != 27 && (k < 0 || k > 127 || std::string("wads cx").find(static_cast<char>(k)) == std::string::npos)) return;
        if (k == 27) { stop("key_esc", clock_.nowMs(), true); exiting = true; return; }
        if (k == 'x') { ++emergencyStops; stop("key_x", clock_.nowMs(), true); return; }
        if (k == 's' || k == ' ') {
            stop(k == 's' ? "key_s" : "key_space", clock_.nowMs()); return;
        }
        if (stopCount != stopsBeforeTick) return;
        if (latched_error || exiting) return;
        const auto received = readAt >= 0 ? readAt : clock_.nowMs();
        if (received > clock_.nowMs() || clock_.nowMs() - received >= static_cast<int64_t>(o_.holdTimeout)) return;
        if (awaitingPress_) {
            // Terminal không báo nhả: phải im phím đủ một hạn giữ kể từ phím cũ cuối cùng.
            const bool released = received - lastHeld_ >= static_cast<int64_t>(o_.holdTimeout);
            lastHeld_ = std::max(lastHeld_, received);
            if (!released || !cameraReady) return;
            awaitingPress_ = false;
        }
        if (k == 'w') {
            if (!cameraReady) { requireFreshPress(); return; }
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
