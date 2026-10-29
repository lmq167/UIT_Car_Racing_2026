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
    explicit ServoStub(std::vector<std::string>& e) : events(e) {}
    bool SetAngle(int v) override { events.push_back("servo:" + std::to_string(v)); return !failing; }
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
    const unsigned char utf8[] = {0xc3, 0xa1, 'w'};
    check(decoder.decode(utf8, sizeof(utf8), 1000) == std::vector<int>{'w'}, "Byte UTF-8 không được bỏ qua");
}
void controlTests() {
    Options o;
    FakeClock clock;
    std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events);
    Control c(o, motor, servo, clock);
    c.halt(); events.clear();
    for (int k : {'w', 'a', 'd', 'y'}) c.key(k);
    check(events.empty() && c.state == State::KHOA, "Khóa vẫn điều khiển được");
    c.key('r'); clock.time = 5001; c.key('y');
    check(c.state == State::KHOA, "Xác nhận quá hạn vẫn mở khóa");
    c.key('r'); c.key('a'); c.key('y');
    check(c.state == State::KHOA, "Phím lạ không hủy xác nhận");
    c.key('R'); clock.time += 5000; c.key('Y');
    check(c.state == State::SAN_SANG, "Không mở khóa R/Y tại biên 5 giây");
    c.key('w', false); check(c.speed == 0, "Chạy khi chưa có ảnh");
    c.key('W'); check(c.speed == 5 && c.state == State::CHAY, "W không chạy đúng tốc độ");
    for (int i = 0; i < 5; ++i) c.key('a');
    check(c.angle == 10, "A phải dương và kẹp 10");
    c.key('s'); check(c.angle == 10 && c.speed == 0, "S phải giữ góc");
    c.key('w'); c.key(' '); check(c.angle == 10 && c.speed == 0, "Space phải giữ góc");
    for (int i = 0; i < 8; ++i) c.key('D');
    check(c.angle == -10, "D phải âm và kẹp -10");
    c.key('C'); check(c.angle == 0, "C không trả lái");
    c.key('w'); clock.time += 799; c.tick(); check(c.speed == 5, "Dừng trước hạn giữ W");
    ++clock.time; c.tick(); check(c.speed == 0 && c.state == State::SAN_SANG, "Không tự dừng đúng hạn W");
    c.key('w'); clock.time += 700; c.key('w'); clock.time += 700; c.tick();
    check(c.speed == 5, "W lặp không gia hạn");
    c.key('a'); events.clear(); c.key('X');
    check(events.size() == 2 && events[0] == "motor:0.000000" && events[1] == "servo:0",
          "X không dừng motor trước servo");
    check(c.emergencyStops == 1 && c.state == State::KHOA, "X không khóa lại");
    c.key('w'); check(c.speed == 0, "W chạy lại sau X");
    c.key('r'); c.key('P'); c.key(0xc3); c.key('?'); c.key('y');
    check(c.state == State::SAN_SANG, "Phím lạ/P/byte UTF-8 không được đổi trạng thái");
    c.key(27); check(c.exiting && c.speed == 0 && c.angle == 0, "Esc không dừng và thoát");
    check(c.maxSpeed == 5 && c.maxAngle == 10, "Sai thống kê lệnh tối đa");
    motor.failing = true; events.clear(); bool failed = false;
    try { c.halt(); } catch (const MotorError&) { failed = true; }
    check(failed && events.size() == 5 && events[1] == "emergency" && events[2] == "emergency" &&
          events[3] == "emergency" && events[4] == "servo:0", "Thiếu 3 lần dừng dự phòng trước trả lái");
    for (const auto& event : events) check(event.find("motor:-") == std::string::npos, "Có lệnh lùi");
    motor.failing = false;
    auto largeStep = parseArgs({"--accept-no-obstacle-brake", "--steer-step-deg", "2147483647"});
    Control large(largeStep, motor, servo, clock);
    large.key('r'); large.key('y'); large.key('a');
    check(large.angle == 10, "Bước rất lớn không được tràn số khi rẽ trái");
    large.key('d'); check(large.angle == -10, "Bước rất lớn không được tràn số khi rẽ phải");
    check(parseArgs({"--accept-no-obstacle-brake", "--max-steer-deg", "0"}).maxSteer == 0,
          "Phải cho phép khóa góc ở 0 độ");
}
void revisedSafetyTests() {
    Options o;
    FakeClock clock;
    std::vector<std::string> events;
    MotorStub motor(events); ServoStub servo(events);
    Control c(o, motor, servo, clock);
    c.key('r', false); c.key('y', false); c.key('w', false);
    check(c.state == State::KHOA && events.empty(), "Không được mở khóa trước khung đầu");
    c.key('r'); c.key('y'); c.key('w');
    clock.time = 800;
    c.key('w');
    check(c.speed == 0 && c.discardPending, "W tồn đọng không được chạy lại ngay sau deadman");
    check(c.stopCount == 1 && c.stops[0].reason == "deadman" && c.stops[0].trigger == 800 &&
          c.stops[0].done == 800, "Nhật ký deadman phải dùng đúng mốc hết hạn");
    c.discardPending = false;
    c.key('w'); clock.time = 810; c.key('s');
    check(c.stopCount == 2 && c.stops[1].reason == "key_s" && c.stops[1].done >= c.stops[1].trigger,
          "S phải sinh đúng một bản ghi dừng");
    c.key('x');
    check(c.stopCount == 3 && c.stops[2].reason == "key_x", "X phải sinh đúng một bản ghi dừng");
    c.latched_error = true; events.clear();
    c.key('r'); c.key('y'); c.key('w'); c.setSpeed(5);
    check(c.state == State::KHOA && events.empty(), "Lỗi đã chốt không được mở khóa hoặc ghi tiến");
    Control badServo(o, motor, servo, clock);
    badServo.key('r'); badServo.key('y'); badServo.key('w');
    servo.failing = true; events.clear(); bool failed = false;
    try { badServo.key('a'); } catch (const ServoError&) { failed = true; }
    check(failed && badServo.latched_error && badServo.speed == 0, "Lỗi servo phải chốt và dừng motor");
    check(events.size() == 5 && events[1] == "motor:0.000000" && events[2] == "emergency" &&
          events[3] == "emergency" && events[4] == "emergency", "Lỗi servo cần đủ 4 lần gọi dừng");
    events.clear(); badServo.key('w');
    check(events.empty(), "W sau lỗi servo không được phát lệnh tiến");
    servo.failing = false;
    Control ring(o, motor, servo, clock);
    for (int i = 0; i < 105; ++i) { clock.time = i; ring.key('s'); }
    check(ring.stopCount == 105 && ring.stops[4].trigger == 104 && ring.stops[5].trigger == 5,
          "Bộ nhớ nhật ký chỉ giữ 100 lần dừng gần nhất");
    Control signal(o, motor, servo, clock);
    motor.clock = &clock; motor.delayMs = 7; clock.time = 1000;
    int notices = 0;
    signal.onStop = [&](const StopRecord&) { ++notices; };
    events.clear(); signal.stop("sigterm", 1000, true, true);
    check(signal.stopCount == 1 && notices == 1 && signal.stops[0].reason == "sigterm",
          "SIGTERM phải phát đúng một bản ghi và một thông báo");
    check(signal.stops[0].trigger == 1000 && signal.stops[0].done == 1007,
          "Độ trễ phải đo tới lúc ghi motor 0 đầu tiên hoàn tất");
    check(events.size() == 2 && events[0] == "motor:0.000000" && events[1] == "servo:0",
          "Nhật ký không được đảo thứ tự motor 0 trước servo 0");
}
}
int main() {
    try { parserTests(); decoderTests(); controlTests(); revisedSafetyTests(); std::cout << "ĐẠT " << checks << " kiểm tra tham số, phím và trạng thái\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
