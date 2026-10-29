#include "options.h"
#include <iostream>

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
    auto o = parseArgs({"--accept-no-obstacle-brake"});
    check(o.speed == 5 && o.maxSteer == 10 && o.holdTimeout == 800 && o.interval == 1000 && o.minFree == 1024,
          "Sai giá trị mặc định");
    check(o.pipeline.find("width=1280,height=720") != std::string::npos, "Sai kích thước camera");
    reject({});
    for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
            {"--speed-cms", "20"}, {"--speed-cms", "-1"}, {"--speed-cms", "nan"},
            {"--speed-cms", "inf"}, {"--speed-cms", "5x"}, {"--max-steer-deg", "30"},
            {"--steer-step-deg", "0"}, {"--max-steer-deg", "-1"}, {"--hold-timeout-ms", "0"},
            {"--hold-timeout-ms", "1.5"}, {"--hold-timeout-ms", "99"}, {"--hold-timeout-ms", "2001"},
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
    check(parseArgs({"--accept-no-obstacle-brake", "--hold-timeout-ms", "100"}).holdTimeout == 100,
          "Phải nhận biên dưới hạn giữ");
    check(parseArgs({"--accept-no-obstacle-brake", "--hold-timeout-ms", "2000"}).holdTimeout == 2000,
          "Phải nhận biên trên hạn giữ");
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
    c.key('W'); check(c.speed == 5 && c.state == State::CHAY, "W mới phải chạy ngay, không mở khóa");
    for (int i = 0; i < 5; ++i) c.key('a');
    check(c.angle == 10, "A phải dương và kẹp 10");
    c.key('s'); check(c.angle == 10 && c.speed == 0 && c.state == State::DUNG, "S phải dừng, giữ góc");
    clock.time += 800; c.key('w'); c.key(' ');
    check(c.angle == 10 && c.speed == 0, "Space phải giữ góc");
    clock.time += 800;
    for (int i = 0; i < 8; ++i) c.key('D');
    check(c.angle == -10, "D phải âm và kẹp -10");
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
    check(c.maxSpeed == 5 && c.maxAngle == 10, "Sai thống kê lệnh tối đa");
    motor.failing = true; events.clear(); bool failed = false;
    try { c.halt(); } catch (const MotorError&) { failed = true; }
    check(failed && events.size() == 5 && events[1] == "emergency" && events[2] == "emergency" &&
          events[3] == "emergency" && events[4] == "servo:0", "Thiếu 3 dừng dự phòng trước trả lái");
    for (const auto& event : events) check(event.find("motor:-") == std::string::npos, "T24 không được thêm lùi");
    motor.failing = false;
    auto largeStep = parseArgs({"--accept-no-obstacle-brake", "--steer-step-deg", "2147483647"});
    Control large(largeStep, motor, servo, clock);
    large.key('a'); check(large.angle == 10, "Bước lớn tràn số bên trái");
    large.key('d'); check(large.angle == -10, "Bước lớn tràn số bên phải");
    check(parseArgs({"--accept-no-obstacle-brake", "--max-steer-deg", "0"}).maxSteer == 0, "Phải nhận góc 0");
}
void revisedSafetyTests() {
    for (const uint64_t hold : {100u, 800u, 2000u}) {
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
    for (int i = 0; i < 105; ++i) { clock.time = i; ring.key('s'); }
    check(ring.stopCount == 105 && ring.stops[4].trigger == 104 && ring.stops[5].trigger == 5, "Sai vòng nhật ký");
    Control signal(o, motor, servo, clock);
    motor.clock = &clock; motor.delayMs = 7; clock.time = 1000;
    int notices = 0; signal.onStop = [&](const StopRecord&) { ++notices; };
    events.clear(); signal.stop("sigterm", 1000, true);
    check(signal.stopCount == 1 && notices == 1 && signal.stops[0].done == 1007, "Sai mốc ghi motor 0");
    check(events == std::vector<std::string>({"motor:0.000000", "servo:0"}), "Nhật ký đảo thứ tự dừng");
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
    check(applied.back().speed == 5 && applied.back().angle == 5 && applied.back().state == State::CHAY,
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

}
int main() {
    try { parserTests(); decoderTests(); controlTests(); revisedSafetyTests(); commandCompletionTests(); std::cout << "ĐẠT " << checks << " kiểm tra tham số, phím và trạng thái\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
