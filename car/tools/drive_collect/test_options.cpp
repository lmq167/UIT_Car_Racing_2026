#define UCR_DRIVE_TEST_HOOKS 1
#include "options.h"
#include <iostream>

namespace drive {
std::vector<std::string> keyTrace;
void hookEvent(const char* event, const std::string& value) {
    if (std::string(event) == "key") keyTrace.push_back(value);
}
}

using namespace drive;
namespace {
int checks = 0;
void check(bool ok, const char* message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
}
Options parseArgs(std::vector<std::string> args) {
    args.insert(args.begin(), "ucr_drive_collect");
    std::vector<char*> argv;
    for (auto& s : args) argv.push_back(&s[0]);
    return parse(static_cast<int>(argv.size()), argv.data());
}
void reject(std::vector<std::string> args) {
    bool rejected = false;
    try { parseArgs(args); } catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "Phải từ chối tham số sai");
}
struct FakeClock : IClock {
    int64_t time = 0;
    int64_t nowMs() const override { return time; }
};
struct MotorStub : IMotor {
    std::vector<std::string>& events;
    bool failing = false;
    FakeClock* clock = nullptr;
    int delayMs = 0;
    explicit MotorStub(std::vector<std::string>& e) : events(e) {}
    void SetSpeedCms(float v) override {
        events.push_back("motor:" + std::to_string(v));
        if (clock) clock->time += delayMs;
    }
    bool EmergencyStop() noexcept override { events.push_back("emergency"); return !failing; }
    bool ok() const override { return !failing; }
};
struct ServoStub : IServo {
    std::vector<std::string>& events;
    bool failing = false;
    std::function<void(int)> beforeWrite;
    explicit ServoStub(std::vector<std::string>& e) : events(e) {}
    bool SetAngle(int v) override {
        if (beforeWrite) beforeWrite(v);
        events.push_back("servo:" + std::to_string(v)); return !failing;
    }
};
void parserTests() {
    check(parseArgs({"--selftest"}).selftest, "Tự kiểm không cần cờ chấp nhận hay tty");
    reject({"--selftest", "--pipeline"});
    reject({"--selftest", "--speed-cms", "61"});
    auto o = parseArgs({"--accept-no-obstacle-brake"});
    check(o.speed == 5 && o.reverseSpeed == 3 && o.steerStep == 5 && o.maxSteer == 40 && o.holdTimeout == 800 && o.interval == 1000 && o.minFree == 1024,
          "Sai giá trị mặc định");
    check(parseArgs({"--accept-no-obstacle-brake", "--speed-cms", "60"}).speed == 60, "Trần 60 cm/s phải được nhận");
    check(!o.recOnStart && o.outRoot == fs::path("data_raw"), "REC mặc định phải tắt, data_raw");
    check(parseArgs({"--accept-no-obstacle-brake", "--rec-on-start"}).recOnStart, "Thiếu cờ REC lúc mở");
    check(o.pipeline.find("width=1280,height=720") != std::string::npos, "Sai kích thước camera");
    reject({});
    for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
            {"--speed-cms", "61"}, {"--speed-cms", "-1"}, {"--speed-cms", "nan"},
            {"--speed-cms", "inf"}, {"--speed-cms", "5x"}, {"--max-steer-deg", "61"},
            {"--steer-step-deg", "0"}, {"--max-steer-deg", "-1"}, {"--hold-timeout-ms", "0"},
            {"--hold-timeout-ms", "1.5"}, {"--hold-timeout-ms", "99"}, {"--hold-timeout-ms", "699"}, {"--hold-timeout-ms", "2001"},
            {"--lock-file", "other.lock"}, {"--interval-ms", "0"}, {"--min-free-mb", "-1"},
            {"--interval-ms", "999999999999999999999999"}, {"--pipeline", "   "}})
        reject({"--accept-no-obstacle-brake", pair.first, pair.second});
    reject({"--accept-no-obstacle-brake", "--unknown"});
    reject({"--accept-no-obstacle-brake", "--out-root"});
    reject({"--accept-no-obstacle-brake", "--speed-cms", "--label", "a"});
    o = parseArgs({"--accept-no-obstacle-brake", "--speed-cms", "2.5", "--out-root", "ảnh thu",
                   "--label", "trái / phải", "--steer-step-deg", "2", "--max-steer-deg", "7"});
    check(o.speed == 2.5 && o.steerStep == 2 && o.maxSteer == 7, "Sai tham số tùy chỉnh");
    check(o.argv.size() == 12 && o.argv[0] == "ucr_drive_collect", "Thiếu argv gốc trong tài liệu phiên");
    check(safeLabel("../trai /phai").find('/') == std::string::npos, "Nhãn lọt đường dẫn");
    check(jsonString("a\n\"\\") == "\"a\\u000a\\\"\\\\\"", "Sai thoát ký tự JSON");
    check(parseArgs({"--accept-no-obstacle-brake", "--hold-timeout-ms", "700"}).holdTimeout == 700,
          "Phải nhận biên dưới hạn giữ");
    check(parseArgs({"--accept-no-obstacle-brake", "--hold-timeout-ms", "2000"}).holdTimeout == 2000,
          "Phải nhận biên trên hạn giữ");
    for (const auto& value : {"5.01", "0", "-1", "nan", "inf", "3x"})
        reject({"--accept-no-obstacle-brake", "--reverse-speed-cms", value});
    for (const auto& value : {"3", "5"})
        check(parseArgs({"--accept-no-obstacle-brake", "--reverse-speed-cms", value}).reverseSpeed == std::stod(value),
              "Sai tốc độ lùi hợp lệ");
    for (const auto& value : {"10", "20", "40"})
        check(parseArgs({"--accept-no-obstacle-brake", "--max-steer-deg", value}).maxSteer == std::stoi(value),
              "Sai trần góc hợp lệ");
    check(!o.preview.no_preview && !o.preview.require_preview && o.preview.preview_scale == 0.5 &&
          o.preview.preview_fps == 10, "Sai mặc định preview");
    o = parseArgs({"--accept-no-obstacle-brake", "--require-preview", "--preview-scale", "0.2", "--preview-fps", "1"});
    check(o.preview.require_preview && o.preview.preview_scale == 0.2 && o.preview.preview_fps == 1,
          "Chưa nối tham số preview vào bộ phân tích chính");
    o = parseArgs({"--accept-no-obstacle-brake", "--no-preview", "--preview-scale", "1", "--preview-fps", "30"});
    check(o.preview.no_preview && o.preview.preview_scale == 1 && o.preview.preview_fps == 30,
          "Sai biên trên tham số preview");
    reject({"--accept-no-obstacle-brake", "--no-preview", "--require-preview"});
    reject({"--accept-no-obstacle-brake", "--require-preview", "--no-preview"});
    for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
            {"--preview-scale", "0.19"}, {"--preview-scale", "1.01"}, {"--preview-scale", "nan"},
            {"--preview-fps", "0"}, {"--preview-fps", "31"}, {"--preview-fps", "1.5"},
            {"--preview-fps", "9999999999999999999999"}})
        reject({"--accept-no-obstacle-brake", pair.first, pair.second});
    reject({"--accept-no-obstacle-brake", "--preview-scale"});
    reject({"--accept-no-obstacle-brake", "--preview-fps"});
}
void decoderTests() {
    KeyDecoder decoder;
    const unsigned char esc[] = {27}, bracket[] = {'['}, arrow[] = {'A'};
    check(decoder.decode(esc, 1, 0).empty(), "Esc không được thoát ngay");
    check(decoder.decode(nullptr, 0, 49).empty(), "Esc thoát trước 50 ms");
    check(decoder.decode(nullptr, 0, 50) == std::vector<int>{27}, "Esc đơn không được nhận");
    decoder.decode(esc, 1, 100);
    decoder.decode(bracket, 1, 110);
    check(decoder.decode(arrow, 1, 120).empty(), "Mũi tên bị coi là A");
    check(decoder.decode(nullptr, 0, 300).empty(), "Mũi tên bị coi là Esc");
    const unsigned char ss3[] = {27, 'O', 'x'}, csi[] = {27, '[', '1', ';', '5', 'D'};
    check(decoder.decode(ss3, sizeof(ss3), 400).empty(), "SS3 bị lọt phím");
    check(decoder.decode(csi, sizeof(csi), 500).empty(), "CSI bị lọt phím");
    const unsigned char cut[] = {27, '['};
    decoder.decode(cut, sizeof(cut), 600);
    check(decoder.decode(nullptr, 0, 900).empty(), "Chuỗi cụt bị coi là Esc");
    check(decoder.decode(arrow, 1, 950) == std::vector<int>{'A'}, "CSI cụt hết hạn phải trả A về phím thường");
    decoder.decode(esc, 1, 960);
    const unsigned char lateTail[] = {'[', 'D'};
    check(decoder.decode(lateTail, sizeof(lateTail), 1020).empty(), "CSI đang chờ bị chốt thành Esc trước khi đọc đuôi");
    const unsigned char cutSs3[] = {27, 'O'}, right[] = {'D'};
    decoder.decode(cutSs3, sizeof(cutSs3), 1030);
    check(decoder.decode(nullptr, 0, 1100).empty(), "SS3 cụt bị coi là Esc");
    check(decoder.decode(right, 1, 1110) == std::vector<int>{'D'}, "SS3 cụt hết hạn phải trả D về phím thường");
    decoder.decode(esc, 1, 1120);
    const unsigned char heldW[] = {'W'}, tail[] = {'[', 'C'};
    check(decoder.decode(heldW, 1, 1125).empty(), "W chen sau Esc không được thành phím lái");
    check(decoder.decode(tail, sizeof(tail), 1130).empty(), "W chen giữa làm lọt đuôi mũi tên");
    decoder.decode(cut, sizeof(cut), 1140);
    decoder.decode(esc, 1, 1150);
    check(decoder.decode(nullptr, 0, 1200) == std::vector<int>{27}, "Esc mới sau CSI cụt phải dừng sau 50 ms");
    decoder.decode(esc, 1, 1210);
    check(decoder.decode(heldW, 1, 1230).empty(), "W chen không được hủy Esc đang chờ");
    check(decoder.decode(heldW, 1, 1260) == std::vector<int>({27, 'W'}), "W liên tục không được trì hoãn Esc quá 50 ms");
    const unsigned char utf8[] = {0xc3, 0xa1, 'w'};
    check(decoder.decode(utf8, sizeof(utf8), 1300) == std::vector<int>{'w'}, "Byte UTF-8 không được bỏ qua");
    const unsigned char unlock[] = {'R'};
    decoder.decode(cut, sizeof(cut), 1400);
    decoder.decode(nullptr, 0, 1451);
    check(decoder.decode(unlock, 1, 1452) == std::vector<int>{'R'}, "CSI cụt hết hạn 50 ms không được nuốt R");
    decoder.decode(cut, sizeof(cut), 1500);
    check(decoder.decode(arrow, 1, 1510).empty(), "CSI phân mảnh trong 10 ms phải nuốt cả A");
    for (const unsigned char stop : std::vector<unsigned char>{3, 'X', 'x', ' '}) {
        KeyDecoder pending;
        pending.decode(esc, 1, 0);
        check(pending.decode(&stop, 1, 10) == std::vector<int>{stop}, "Phím dừng sau Esc đang chờ bị nuốt");
        pending.discard();
        check(pending.decode(nullptr, 0, 50) == std::vector<int>{27}, "Phím dừng/xả lô làm mất Esc đang chờ");
    }
    // S nay là lùi: chen sau Esc phải bị bỏ như W, không trì hoãn hay làm mất Esc.
    for (const unsigned char held : std::vector<unsigned char>{'S', 's'}) {
        KeyDecoder pending;
        pending.decode(esc, 1, 0);
        check(pending.decode(&held, 1, 10).empty(), "S chen sau Esc không được thành phím lùi");
        check(pending.decode(nullptr, 0, 50) == std::vector<int>{27}, "S chen làm mất Esc đang chờ");
    }
    KeyDecoder interrupted;
    interrupted.decode(cut, sizeof(cut), 0);
    const unsigned char ctrlC[] = {3};
    check(interrupted.decode(ctrlC, 1, 10) == std::vector<int>{3}, "CSI cụt không được nuốt Ctrl+C");
    for (size_t position = 0; position <= 8; ++position) {
        KeyDecoder burst;
        std::string input = "wAdCWWWW";
        input.insert(position, 1, char(27));
        burst.decode(reinterpret_cast<const unsigned char*>(input.data()), input.size(), 0);
        burst.discard(); // Như Terminal sau khi chặn lô hoặc xử lý phím dừng.
        check(burst.decode(nullptr, 0, 49).empty(), "Xả lô làm Esc thoát sớm");
        check(burst.decode(nullptr, 0, 50) == std::vector<int>{27}, "Esc trong lô dồn bị mất sau xả");
    }
}
// T24/overnight A: DUNG, W mới, phím cũ và đường dừng theo đồng hồ giả.
void controlTests() {
    Options o;
    FakeClock clock;
    std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events);
    Control c(o, motor, servo, clock);
    check(c.state == State::DUNG && c.speed == 0 && c.angle == 0, "Mở phải là DUNG");
    c.key('r'); c.key('y'); check(events.empty(), "R/Y phải bị bỏ qua");
    c.key('W'); check(c.speed == 5 && c.state == State::CHAY_TIEN, "W mới phải chạy ngay, không mở khóa");
    for (int i = 0; i < 10; ++i) c.key('a');
    check(c.angle == 40, "A phải dương và kẹp 40");
    c.key(' '); check(c.angle == 40 && c.speed == 0 && c.state == State::DUNG, "Space phải dừng, giữ góc");
    clock.time += 800; c.key('w'); c.key(' ');
    check(c.angle == 40 && c.speed == 0, "Space phải giữ góc");
    clock.time += 800;
    for (int i = 0; i < 20; ++i) c.key('D');
    check(c.angle == -40, "D phải âm và kẹp -40");
    c.key('C'); check(c.angle == 0, "C không trả lái");
    c.key('w'); clock.time += 799; c.tick(); check(c.speed == 5, "Dừng trước hạn W");
    ++clock.time; c.tick(); check(c.speed == 0 && c.state == State::DUNG, "Deadman không về DUNG");
    clock.time += 800; c.key('w'); clock.time += 700; c.key('w'); clock.time += 700; c.tick();
    check(c.speed == 5, "W lặp không gia hạn");
    c.key('a'); events.clear(); c.key('X');
    check(events == std::vector<std::string>({"motor:0.000000", "servo:0"}), "X phải motor 0 trước servo 0");
    check(c.emergencyStops == 1 && c.state == State::DUNG, "X phải về DUNG");
    c.key('w'); check(c.speed == 0, "W cũ chạy lại sau X");
    clock.time += 800; c.key('w'); check(c.speed == 5, "W mới sau X không cần R/Y");
    c.key(27); check(c.exiting && c.speed == 0 && c.angle == 0, "Esc phải dừng và thoát");
    check(c.maxSpeed == 5 && c.maxAngle == 40, "Sai thống kê lệnh tối đa");
    motor.failing = true; events.clear(); bool failed = false;
    try { c.halt(); } catch (const MotorError&) { failed = true; }
    check(failed && events.size() == 5 && events[1] == "emergency" && events[2] == "emergency" &&
          events[3] == "emergency" && events[4] == "servo:0", "Thiếu 3 dừng dự phòng trước trả lái");
    for (const auto& event : events) check(event.find("motor:-") == std::string::npos, "Đường dừng không được phát lệnh âm");
    motor.failing = false;
    auto largeStep = parseArgs({"--accept-no-obstacle-brake", "--steer-step-deg", "2147483647"});
    Control large(largeStep, motor, servo, clock);
    large.key('a'); check(large.angle == 40, "Bước lớn tràn số bên trái");
    large.key('d'); check(large.angle == -40, "Bước lớn tràn số bên phải");
    check(parseArgs({"--accept-no-obstacle-brake", "--max-steer-deg", "0"}).maxSteer == 0, "Phải nhận góc 0");
    for (const int limit : {0, 10, 40, 60}) {
        auto custom = parseArgs({"--accept-no-obstacle-brake", "--max-steer-deg", std::to_string(limit)});
        check(custom.maxSteer == limit && custom.steerStep == 5, "Sai trần tùy chọn hoặc bước mặc định");
        Control steering(custom, motor, servo, clock);
        for (int i = 0; i < 15; ++i) {
            steering.key('A');
            check(steering.angle == std::min(limit, (i + 1) * 5), "A không tăng đúng bước/kẹp trần");
        }
        for (int i = 0; i < 30; ++i) {
            steering.key('D');
            check(steering.angle == std::max(-limit, limit - (i + 1) * 5), "D không giảm đúng bước/kẹp trần");
        }
        steering.key('C'); check(steering.angle == 0, "C phải trả 0 với mọi trần");
    }
    reject({"--accept-no-obstacle-brake", "--max-steer-deg", "60.01"});
}
void revisedSafetyTests() {
    for (const uint64_t hold : {700u, 800u, 2000u}) {
        for (const char* reason : {"key_x", "frame_stale", "preview_closed", "preview_died", "dir_change"}) {
            Options o; o.holdTimeout = hold;
            FakeClock clock;
            std::vector<std::string> events;
            MotorStub motor(events); ServoStub servo(events);
            Control c(o, motor, servo, clock);
            c.key('w'); clock.time = 10; c.stop(reason, clock.time, true);
            check(c.state == State::DUNG && c.speed == 0 && c.angle == 0, "Sự cố phải về DUNG");
            check(c.stopCount == 1 && c.stops[0].reason == reason && c.stops[0].trigger == 10,
                  "Sai nhật ký sự cố");
            events.clear();
            clock.time += 660; c.key('w');
            check(c.speed == 0 && events.empty(), "Lần tự lặp đầu 660 ms tự mở chốt sau sự cố");
            // Phím lặp kéo dài nhiều hạn giữ: chờ từ sự cố thôi sẽ không đủ.
            for (int i = 0; i < 12; ++i) {
                clock.time += hold / 2; c.key('w');
                check(c.speed == 0 && events.empty(), "W giữ cũ làm chạy lại sau sự cố");
            }
            const auto last = clock.time;
            clock.time += hold - 1; c.key('w');
            check(c.speed == 0, "Nhận W trước khi im đủ hạn");
            clock.time += hold; c.key('w', true, last); // Lô cũ không được làm mới mốc nhận.
            check(c.speed == 0, "Lô W hết tuổi vẫn được chạy");
            c.key('W'); check(c.speed == 5, "Không nhận lần nhấn mới tại biên hạn giữ");
            clock.time += hold; c.key('w');
            check(c.speed == 0 && c.stops[c.stopCount-1].reason == "deadman", "W đúng hạn che deadman");
        }
    }
    Options o;
    FakeClock clock;
    std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events);
    Control c(o, motor, servo, clock);
    c.key('w', false); clock.time = 20; c.key('w', true);
    check(c.speed == 0, "W trước ảnh đầu tự chạy khi ảnh về");
    clock.time += 800; c.key('w'); check(c.speed == 5, "W mới sau khung đầu không chạy");
    c.latched_error = true; c.stop("motor_fail", clock.time, true); events.clear();
    clock.time += 800; c.key('w'); c.setSpeed(5);
    check(events.empty() && c.state == State::DUNG, "Lỗi chốt vẫn phát lệnh tiến");
    Control badServo(o, motor, servo, clock);
    badServo.key('w'); servo.failing = true; events.clear(); bool failed = false;
    try { badServo.key('a'); } catch (const ServoError&) { failed = true; }
    check(failed && badServo.latched_error && badServo.speed == 0, "Lỗi servo phải chốt và dừng");
    check(events.size() == 5 && events[1] == "motor:0.000000" && events[2] == "emergency" &&
          events[3] == "emergency" && events[4] == "emergency", "Lỗi servo thiếu bốn lần dừng");
    events.clear(); clock.time += 800; badServo.key('w'); check(events.empty(), "W sau lỗi servo vẫn chạy");
    servo.failing = false;
    Control ring(o, motor, servo, clock);
    for (int i = 0; i < 105; ++i) { clock.time = i; ring.key(' '); }
    check(ring.stopCount == 105 && ring.stops[4].trigger == 104 && ring.stops[5].trigger == 5, "Sai vòng nhật ký");
    Control signal(o, motor, servo, clock);
    motor.clock = &clock; motor.delayMs = 7; clock.time = 1000;
    int notices = 0; signal.onStop = [&](const StopRecord&) { ++notices; };
    events.clear(); signal.stop("sigterm", 1000, true);
    check(signal.stopCount == 1 && notices == 1 && signal.stops[0].done == 1007, "Sai mốc ghi motor 0");
    check(events == std::vector<std::string>({"motor:0.000000", "servo:0"}), "Nhật ký đảo thứ tự dừng");
}
void burstTests() {
    for (const uint64_t hold : {700u, 701u, 800u, 2000u}) {
        const auto half = (hold+1)/2;
        KeyBurstGuard guard(hold, 0);
        check(!guard.reject({'w','A','d','C'}, {}, half - 1), "Không được chặn khi chưa im H/2");
        check(!guard.reject({'w','a','d'}, {}, hold - 1), "Ba phím chưa phải lô dồn");
        check(guard.reject({}, {'w','A','d','C'}, hold + half - 1), "Phải chặn bốn phím preview tại biên H/2");
        check(!guard.reject({'w','a','d'}, {'C'}, hold*2), "Không cộng hai lần đọc tty/preview thành bốn");
        check(!guard.reject({'W'}, {}, hold*4), "Nhấn đơn sau im vẫn nhận");
        check(!guard.reject({'w','w','w','w'}, {}, hold*4 + 10), "Giữ phím liên tục vẫn nhận");
        check(guard.reject({'X','w','A','d','C',' ',27,3}, {}, hold*5), "Phím dừng không làm lọt lô dồn");
        check(guard.reject({'s','S','w','a'}, {}, hold*7), "S (lùi) phải được đếm vào lô dồn");
        Options o; o.holdTimeout = hold;
        FakeClock clock;
        std::vector<std::string> events;
        MotorStub motor(events); ServoStub servo(events);
        Control c(o, motor, servo, clock);
        c.rejectKeyBurst(clock.time);
        check(events.empty() && c.stopCount == 0, "Lô khi DUNG chỉ ghi vết, không có sự kiện dừng");
        clock.time = hold - 1; c.key('w');
        check(c.speed == 0, "Lô khi DUNG phải yêu cầu nhấn mới");
        clock.time += hold; c.key('w');
        check(c.speed == 5, "W mới sau H không được mất");
        events.clear(); clock.time += hold/2; c.rejectKeyBurst(clock.time);
        check(c.speed == 0 && c.state == State::DUNG && c.stops[c.stopCount-1].reason == "key_burst",
              "Lô khi CHAY phải dừng và ghi key_burst");
        check(events == std::vector<std::string>({"motor:0.000000", "servo:0"}), "Lô phải motor 0 trước servo 0");
        clock.time += hold; c.key('w'); check(c.speed == 5, "Nhấn mới sau lô CHAY không được mất");
    }
    for (const int key : std::vector<int>{27, 'X', ' '}) {
        Options o;
        FakeClock clock;
        std::vector<std::string> events;
        MotorStub motor(events); ServoStub servo(events);
        Control c(o, motor, servo, clock);
        c.key('w');
        clock.time = o.holdTimeout;
        c.key(key);
        check(c.speed == 0 && c.state == State::DUNG, "Phím an toàn tại hạn deadman không dừng");
        check(c.stops[c.stopCount-1].reason == (key == 27 ? "key_esc" : key == 'X' ? "key_x" : "key_space"),
              "Deadman nuốt phím dừng/thoát cùng lượt");
        check(c.exiting == (key == 27) && c.emergencyStops == (key == 'X' ? 1u : 0u),
              "Thiếu thoát/bộ đếm dừng khẩn tại hạn deadman");
    }
}
void batchKeyTests() {
    keyTrace.clear();
    for (const int key : std::string("wAdC"))
        check(acceptBatchKey(key, false), "Lô chưa gặp phím dừng phải giữ vết để kiểm D98/D99/D101");
    check(keyTrace == std::vector<std::string>({"w", "A", "d", "C"}), "Mất vết phân loại lô dồn");
    for (const int stop : std::vector<int>{' ', 'x', 'X'}) {
        Options o;
        FakeClock clock;
        std::vector<std::string> events;
        MotorStub motor(events); ServoStub servo(events);
        Control c(o, motor, servo, clock);
        c.key('w'); c.key('a');
        check(acceptBatchKey(stop, false), "Phím dừng đầu lô phải được nhận");
        c.key(stop);
        check(c.discardPending && c.speed == 0, "Phím dừng chưa yêu cầu bỏ đuôi lô");
        events.clear(); keyTrace.clear();
        // S (lùi) cũng là đuôi chuyển động phải bỏ.
        for (const int key : std::string("wWsSaAdDcC")) {
            if (acceptBatchKey(key, true)) c.key(key);
        }
        check(keyTrace.empty(), "Đuôi chuyển động sau dừng vẫn phát sự kiện key");
        check(events.empty(), "Đuôi chuyển động sau dừng vẫn ghi motor/servo");
        for (const int key : std::vector<int>{' ', 'x', 'X', 27, 3}) {
            check(acceptBatchKey(key, true), "Quét đuôi lô làm mất phím an toàn");
            if (key != 3) c.key(key); // Ctrl+C được main xử lý bằng đường thoát riêng.
        }
        check(keyTrace == std::vector<std::string>({" ", "x", "X", "ESC", std::string(1, char(3))}),
              "Vết phím an toàn sau dừng bị thiếu hoặc lẫn phím chuyển động");
        check(c.exiting && c.emergencyStops >= 2 && c.angle == 0,
              "Phím an toàn sau dừng chưa thoát/dừng khẩn/trả lái");
    }
    keyTrace.clear();
    check(acceptBatchKey('W', false) && keyTrace == std::vector<std::string>{"W"},
          "Lô mới sau dừng vẫn bị bỏ ngầm");
}
void drainTests() {
    for (const int safe : std::vector<int>{27, 3, 'X', ' '}) {
        Options o;
        FakeClock clock;
        std::vector<std::string> events;
        MotorStub motor(events); ServoStub servo(events);
        Control c(o, motor, servo, clock);
        // Mô phỏng pollKeys lấy 64 byte đầu, phím thứ 65 vẫn nằm trong pipe.
        std::vector<int> keys(64, 'c');
        clock.time = 400;
        check(c.keyBurst({}, keys, clock.time), "Lô 64 phím phải bị chặn");
        c.rejectKeyBurst(clock.time);
        int reads = 0;
        check(drainSafetyKeys(keys, [&]() {
            return reads++ == 0 ? std::vector<int>{safe, 'W', 'S', 's', 'a', 'd', 'c'} : std::vector<int>{};
        }, [&]() { return clock.time; }), "Không đọc hết phần hàng còn lại");
        check(reads == 2 && keys.size() == 65 && keys.back() == safe, "Mất phím thứ 65 hoặc giữ chuyển động cũ");
        for (int key : keys) if (safetyKey(key) && key != 3) c.key(key);
        check(c.speed == 0 && events.end() == std::find(events.begin(), events.end(), "motor:5.000000"),
              "Lô bị xả vẫn có tốc độ dương");
        if (safe != 3) {
            check(c.stops[c.stopCount-1].reason == (safe == 27 ? "key_esc" : safe == 'X' ? "key_x" : "key_space"),
                  "Sai lý do phím thứ 65");
        }
        // Đúng lượt tick deadman, sau đó mới đọc và lọc hàng chưa đọc.
        Control deadman(o, motor, servo, clock);
        deadman.key('w'); clock.time += o.holdTimeout; deadman.tick();
        std::vector<int> pending;
        reads = 0;
        check(drainSafetyKeys(pending, [&]() {
            return reads++ == 0 ? std::vector<int>{'W', safe} : std::vector<int>{};
        }, [&]() { return clock.time; }) && pending == std::vector<int>{safe}, "Deadman làm mất phím an toàn đang chờ");
        events.clear();
        if (safe != 3) deadman.key(pending[0]);
        check(deadman.speed == 0 && deadman.exiting == (safe == 27), "Esc tại tick deadman không thoát");
        check(std::find(events.begin(), events.end(), "motor:5.000000") == events.end(), "Chạy lại sau deadman");
    }
    for (int bound = 0; bound < 3; ++bound) {
        std::vector<int> keys;
        int calls = 0;
        int64_t time = 0;
        const bool complete = drainSafetyKeys(keys, [&]() {
            ++calls;
            if (bound == 2) time += 5;
            return std::vector<int>(bound == 1 ? 65536 : 1, 'w');
        }, [&]() { return time; });
        check(!complete && keys.empty(), "Không chặn hàng vô hạn theo lượt/byte/thời gian");
        check(calls == (bound == 0 ? 256 : 1), "Vượt trần đọc hàng phím");
    }
    KeyDecoder tty;
    const unsigned char bytes[] = {'W', 27};
    tty.decode(bytes, sizeof(bytes), 0);
    std::vector<int> pending;
    drainSafetyKeys(pending, [&]() { return tty.decode(nullptr, 0, 10); }, []() { return 10; });
    check(tty.decode(nullptr, 0, 50) == std::vector<int>{27}, "Đọc xả làm mất Esc tty chưa đủ 50 ms");
}
void commandCompletionTests() {
    Options o;
    FakeClock clock;
    std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events);
    Control c(o, motor, servo, clock);
    struct Applied { int64_t ns; float speed; int angle; State state; };
    std::vector<Applied> applied;
    c.onCommand = [&](int64_t ns, float speed, int angle, State state) {
        applied.push_back({ns, speed, angle, state});
    };
    c.key('w'); c.key('a');
    check(applied.back().speed == 5 && applied.back().angle == 5 && applied.back().state == State::CHAY_TIEN,
          "Thiếu công bố lệnh đã hoàn tất");
    servo.beforeWrite = [&](int angle) {
        check(angle == 0 && applied.back().speed == 0 && applied.back().angle == 5 &&
              applied.back().state == State::DUNG, "Motor 0 phải có nhãn DUNG trước trả lái");
    };
    c.stop("key_x", clock.time, true);
    check(applied.back().speed == 0 && applied.back().angle == 0 && applied.back().state == State::DUNG,
          "Thiếu công bố dừng/trả lái");
    servo.beforeWrite = {};
    const auto beforeHalt = applied.size(); c.halt();
    check(applied.size() == beforeHalt + 2, "halt thiếu công bố motor 0/servo 0");
    clock.time += 800; c.key('w'); clock.time += 800; c.tick();
    check(applied.back().speed == 0 && applied.back().state == State::DUNG, "Deadman thiếu công bố DUNG");
    clock.time += 800; c.key('w'); servo.failing = true;
    bool failed = false; try { c.key('a'); } catch (const ServoError&) { failed = true; }
    check(failed && applied.back().speed == 0 && applied.back().angle == 0 &&
          applied.back().state == State::DUNG, "Sai công bố sau lỗi servo");
    for (size_t i = 1; i < applied.size(); ++i)
        check(applied[i].ns >= applied[i - 1].ns, "Mốc lệnh phải tăng đơn điệu");
    motor.failing = true; const auto before = applied.size();
    try { c.setSpeed(0); } catch (const MotorError&) {}
    check(applied.size() == before, "Công bố lệnh ghi lỗi như thành công");
}

void recordingTests() {
    Options o; FakeClock clock; std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events); Control c(o, motor, servo, clock);
    int accepted = 0, rejected = 0;
    c.onRecording = [&] { ++accepted; };
    c.onRecordingRejected = [&] { ++rejected; };
    c.key('P'); clock.time = 1000; c.key('p');
    check(accepted == 2 && events.empty(), "P lúc dừng không được ghi motor/servo");
    clock.time = 2000; c.key('w'); const auto before = events.size(); c.key('p');
    check(rejected == 1 && accepted == 2 && events.size() == before, "P lúc chạy phải bị bỏ");
    c.key(' '); const auto stopped = events.size(); c.key('P');
    check(rejected == 2 && events.size() == stopped, "P khi W còn giữ phải bị bỏ");
    clock.time = 2799; c.key('P'); check(accepted == 2, "P trước hạn nhả W");
    clock.time = 2800; c.key('P'); check(accepted == 3, "P sau hạn nhả W");
    clock.time = 2910; c.key('w'); c.key(' '); clock.time = 3000; c.key('w');
    clock.time = 3710; c.key('p'); check(accepted == 3, "W bị chặn vẫn là W đang giữ");
    clock.time = 3800; c.key('p'); check(accepted == 4, "Không nhận P sau nhả W bị chặn");
    clock.time = 4000; c.noteW('W'); // W bị vòng nhập bỏ trước Control::key.
    clock.time = 4799; c.key('p'); check(accepted == 4, "P nhận khi W đuôi lô chưa nhả");
    clock.time = 4800; c.key('p'); check(accepted == 5, "P không nhận sau nhả W đuôi lô");
    clock.time = 4900; c.rejectKeyBurst(clock.time); c.key('p');
    check(accepted == 5, "P nhận ngay sau lô bị xả có W chưa đọc");
    clock.time = 6000; c.noteW('S'); // S (lùi) giữ cũng chặn P như W.
    clock.time = 6799; c.key('p'); check(accepted == 5, "P nhận khi S còn giữ");
    clock.time = 6800; c.key('p'); check(accepted == 6, "P không nhận sau nhả S");
    KeyDecoder decoder; const unsigned char bytes[] = {'P', 'p'};
    check(decoder.decode(bytes, 2, 0) == std::vector<int>({'P', 'p'}), "TTY mất P/p");
    check(!acceptBatchKey('P', true) && !acceptBatchKey('p', true), "Đuôi sau dừng còn P");
    std::vector<int> retained; int reads = 0;
    check(drainSafetyKeys(retained, [&] { return reads++ ? std::vector<int>{} : std::vector<int>{'p', 'P', 'x', 27, 3}; }, [] { return 0; }), "Xả lô P lỗi");
    check(retained == std::vector<int>({'x', 27, 3}), "Xả lô phải bỏ P, giữ phím an toàn");
}

void recordingRepeatTests() {
    for (const uint64_t hold : {700u, 800u, 1000u, 2000u}) {
        Options o; o.holdTimeout = hold;
        FakeClock clock; std::vector<std::string> events;
        MotorStub motor(events); ServoStub servo(events); Control c(o, motor, servo, clock);
        int toggles = 0;
        c.onRecording = [&] { ++toggles; };
        const auto window = static_cast<int64_t>(std::max<uint64_t>(hold, 1000));
        c.key('P', true, 0);
        for (clock.time = 0; clock.time < window; clock.time += 50) {
            c.key('p', true, clock.time); // Cùng Control cho tty và preview.
            check(toggles == 1 && events.empty(), "P tự lặp đổi REC hoặc phát lệnh xe");
        }
        clock.time = window - 1; c.key('P');
        check(toggles == 1, "P trước biên chống lặp vẫn được nhận");
        clock.time = window; c.key('p');
        check(toggles == 2, "P đúng biên không được nhận; phím bỏ qua đã gia hạn cửa sổ");
        clock.time = 2 * window + 1; c.key('P');
        check(toggles == 3 && events.empty(), "P sau cửa sổ không được nhận độc lập W/motor/servo");
    }
}
}
int main() {
    try { recordingRepeatTests(); recordingTests(); parserTests(); decoderTests(); controlTests(); revisedSafetyTests(); burstTests(); batchKeyTests(); drainTests(); commandCompletionTests(); std::cout << "ĐẠT " << checks << " kiểm tra tham số, phím và trạng thái\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
