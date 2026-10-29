#include "options.h"
#include <iostream>

using namespace drive;
namespace {
void need(bool ok, const char* tag) { if (!ok) throw std::runtime_error(tag); }
struct Clock : IClock {
    int64_t now = 0;
    int64_t nowMs() const override { return now; }
};
struct Motor : IMotor {
    Clock& clock;
    std::vector<float> speeds;
    int delay = 0;
    bool failing = false;
    explicit Motor(Clock& c) : clock(c) {}
    void SetSpeedCms(float v) override { speeds.push_back(v); clock.now += delay; }
    bool EmergencyStop() noexcept override { return !failing; }
    bool ok() const override { return !failing; }
};
struct Servo : IServo {
    std::vector<int> angles;
    bool failing = false;
    bool SetAngle(int v) override { angles.push_back(v); return !failing; }
};
struct Rig {
    Options options;
    Clock clock;
    Motor motor{clock};
    Servo servo;
    Control control{options, motor, servo, clock};
};
void reverseParser() {
    for (const char* value : {"5.01", "0", "-1", "nan", "inf"}) {
        std::vector<std::string> args{"drive", "--accept-no-obstacle-brake", "--reverse-speed-cms", value};
        std::vector<char*> argv;
        for (auto& arg : args) argv.push_back(&arg[0]);
        bool rejected = false;
        try { parse(static_cast<int>(argv.size()), argv.data()); }
        catch (const std::runtime_error&) { rejected = true; }
        need(rejected, "reverse_reject");
    }
}
void directionPause() {
    Rig r;
    r.control.key('w');
    r.clock.now = 800; r.control.tick();
    r.clock.now = 1600;
    // Đồng hồ giả kéo dài ghi 0 để tách hạn nhả khỏi mốc ghi XONG 0; không đo độ trễ thật.
    r.motor.delay = 750;
    r.control.key('s');
    need(r.control.stops[r.control.stopCount - 1].reason == "dir_change", "direction_setup");
    r.motor.delay = 0;
    r.clock.now = 2400; r.control.key('s');
    need(r.control.speed == 0, "direction_pause");
    r.clock.now = 3200; r.control.key('s');
    need(r.control.speed == -3 && r.control.state == State::CHAY_LUI, "direction_fresh");
}
void contract() {
    reverseParser();
    for (const uint64_t hold : {700u, 800u, 2000u}) {
        for (const bool reverse : {false, true}) {
            Rig r; r.options.holdTimeout = hold;
            const char key = reverse ? 'S' : 'W';
            r.control.key(key);
            need(r.control.speed == (reverse ? -3 : 5), "reverse_motion");
            need(r.control.state == (reverse ? State::CHAY_LUI : State::CHAY_TIEN), "reverse_state");
            r.clock.now = hold - 1; r.control.tick();
            need(r.control.speed != 0, "deadman_early");
            r.clock.now = hold; r.control.tick();
            need(r.control.speed == 0 && r.control.state == State::DUNG, "reverse_deadman");
            r.control.key(key); need(r.control.speed == 0, "fresh_press");
            r.clock.now += hold; r.control.key(key);
            need(r.control.speed != 0, "fresh_resume");
        }
    }
    for (const char first : {'w', 's'}) {
        Rig r;
        r.control.key(first); r.control.key('a');
        r.clock.now = 10; r.control.key(first == 'w' ? 's' : 'w');
        need(r.control.state == State::DUNG && r.control.speed == 0 && r.control.angle == 5,
             "reverse_conflict");
        need(r.control.stops[0].reason == "reverse_conflict", "conflict_reason");
        for (int i = 0; i < 6; ++i) {
            r.clock.now += 200; r.control.key(i % 2 ? 's' : 'w');
            need(r.control.speed == 0, "conflict_held");
        }
        r.clock.now += 800; r.control.key('s'); need(r.control.speed == -3, "conflict_release");
    }
    for (const char key : {' ', 'x', char(27)}) {
        Rig r; r.control.key('s'); r.control.key('a');
        r.control.key(key);
        need(r.control.speed == 0 && r.motor.speeds.back() == 0, "reverse_stop");
        need(r.control.stops[0].reason != "key_s", "s_not_stop");
        need(key == ' ' ? r.control.angle == 5 : r.control.angle == 0, "stop_steer");
    }
    Rig stale;
    stale.control.key('s', false);
    stale.clock.now = 10; stale.control.key('s');
    need(stale.motor.speeds.empty(), "reverse_camera");
    stale.clock.now = 810; stale.control.key('s');
    need(stale.control.speed == -3, "reverse_camera_resume");
    stale.control.stop("frame_stale", 810, true);
    stale.clock.now = 820; stale.control.key('s');
    need(stale.control.speed == 0, "reverse_stale_restart");
    Rig guard;
    guard.control.setSpeed(-3); need(guard.motor.speeds.empty(), "negative_in_dung");
    guard.control.key('w'); guard.control.setSpeed(-3);
    need(guard.control.speed == 5, "negative_in_forward");
    for (const bool motorFault : {true, false}) {
        Rig r;
        r.control.onCommand = [](int64_t, float speed, int, State state) {
            need((speed < 0) == (state == State::CHAY_LUI), "completed_reverse_state");
        };
        r.control.key('s');
        r.motor.failing = motorFault; r.servo.failing = !motorFault;
        bool failed = false;
        try { if (motorFault) r.control.key(' '); else r.control.key('a'); }
        catch (const MotorError&) { failed = motorFault; }
        catch (const ServoError&) { failed = !motorFault; }
        need(failed && r.control.latched_error && r.control.speed == 0 && r.control.state == State::DUNG,
             "reverse_fault");
        need(r.motor.speeds.back() == 0, "fault_motor_zero");
        const auto count = r.motor.speeds.size();
        r.clock.now += 800; r.control.key('s');
        need(r.motor.speeds.size() == count, "fault_reverse_restart");
    }
    for (const int cap : {10, 20, 40}) {
        Rig r; r.options.maxSteer = cap;
        for (int i = 0; i < 20; ++i) r.control.key('a');
        need(r.control.angle == cap, "left_limit");
        for (int i = 0; i < 40; ++i) r.control.key('d');
        need(r.control.angle == -cap, "right_limit");
    }
    directionPause();
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "direction_pause") directionPause();
        else if (argc == 2 && std::string(argv[1]) == "reverse_reject") reverseParser();
        else contract();
        std::cout << "ĐẠT hợp đồng tiến/lùi, nhấn mới, xung đột, ảnh và nghỉ đổi hướng\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
