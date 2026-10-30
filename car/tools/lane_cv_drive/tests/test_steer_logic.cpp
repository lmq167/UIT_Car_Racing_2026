// T27: kỳ vọng chỉ từ T27_spec.md, drive_collect.md và T27_cases.md.
// Không đọc cài đặt; giao diện giả định được ghi ở đầu T27_cases.md.
// Biên dịch: g++ -std=c++17 -Wall -Wextra test_steer_logic.cpp -o test_steer_logic
// Mỗi nhóm tương ứng ca L; assert hỏng làm tiến trình trả mã khác 0.
#ifdef NDEBUG
#error "Test T27 can assert; khong bien dich voi NDEBUG"
#endif
#include "../steer_logic.h"
#include <cassert>
#include <initializer_list>
#include <limits>

using namespace lane_cv_drive;

static double magnitude(double value) {
    return value < 0 ? -value : value;
}

static bool near(double actual, double expected) {
    return magnitude(actual - expected) <= 1e-8;
}

static Options assist_options() {
    Options options{};
    options.steer_assist = true;
    options.speed_cms = 5;
    options.max_steer_deg = 40;
    options.min_conf = 0.6;
    options.lost_lane_frames = 3;
    options.max_steer_rate_deg_s = 20;
    options.hold_timeout_ms = 800;
    return options;
}

static TickInput tick(long long now_ms) {
    TickInput input{};
    input.now_ms = now_ms;
    input.frame_age_ms = 0;
    input.advice = {true, 0, 0.9};
    input.w_deadman_valid = true;
    input.stop_key = StopKey::None;
    return input;
}

static Decision drive(SteerGate& gate, long long now_ms, double angle = 0) {
    TickInput input = tick(now_ms);
    input.advice.angle_deg = angle;
    return gate.step(input);
}

// L01–L03: biên tùy chọn, không dùng trần 10 của công cụ cũ.
static void options_cases() {
    Options defaults{};
    assert(near(defaults.speed_cms, 5) && "L01 speed mac dinh");
    assert(near(defaults.max_steer_deg, 10) && "L01 goc mac dinh");
    assert(!defaults.steer_assist && "L01 mac dinh GOI Y");
    assert(validateOptions(defaults) && "L01 options mac dinh hop le");

    const double speeds[] = {-0.01, 0, 5, 5.01};
    for (int i = 0; i < 4; ++i) {
        Options options{};
        options.speed_cms = speeds[i];
        assert(validateOptions(options) == (i == 1 || i == 2)
               && "L02 speed chi trong [0,5]");
    }
    const double angles[] = {-0.01, 0, 10, 40, 40.01};
    for (int i = 0; i < 5; ++i) {
        Options options{};
        options.max_steer_deg = angles[i];
        assert(validateOptions(options) == (i >= 1 && i <= 3)
               && "L03 max steer chi trong [0,40]");
    }
}

// L04/M1: GỢI Ý luôn không điều khiển, kể cả nhánh lỗi làn/ảnh.
static void suggestion_case() {
    Options options{};
    for (bool has_lane : {false, true}) {
        for (long long age : {0LL, 900LL}) {
            for (bool w_valid : {false, true}) {
                SteerGate gate(options);
                for (int n = 0; n <= options.lost_lane_frames; ++n) {
                    TickInput input = tick(n * 1000);
                    input.advice = {has_lane, n % 2 == 0 ? 35.0 : -35.0, 0.9};
                    input.frame_age_ms = age;
                    input.w_deadman_valid = w_valid;
                    const auto output = gate.step(input);
                    assert(output.mode == Mode::GOI_Y && "L04 mode GOI Y");
                    assert(!output.command_servo && "L04 M1 ke ca loi khong servo");
                    assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
                           && "L04 khong lenh chuyen dong");
                    assert(output.advice.has_lane == input.advice.has_lane
                           && near(output.advice.angle_deg, input.advice.angle_deg)
                           && near(output.advice.conf, input.advice.conf)
                           && "L04 giu advice ke ca khong lan va anh cu");
                }
            }
        }
    }
}

// L05–L07: L06 theo duyệt, DEV công bố trả 0 hoặc không lệnh servo.
static void deadman_and_speed_cases() {
    Options options = assist_options();
    SteerGate gate(options);
    TickInput input = tick(0);
    input.w_deadman_valid = false;
    const auto idle = gate.step(input);
    assert(near(idle.motor_cms, 0) && "L05 chua W thi dung");
    assert((!idle.command_servo || near(idle.servo_deg, 0))
           && "L05 chua W khong lai khac 0");
    const auto running = drive(gate, 1000, 10);
    assert(running.mode == Mode::LAI_GOC && running.command_servo
           && near(running.motor_cms, 5) && "L05 W hop le cho lai");
    assert(magnitude(running.servo_deg) > 0 && "L06 thiet lap dang co goc");
    input = tick(1001);
    input.w_deadman_valid = false;
    input.advice.angle_deg = -35;
    const auto stopped = gate.step(input);
    assert(near(stopped.motor_cms, 0) && stopped.need_new_press
           && "L06 het deadman dung va chot");
    assert((!stopped.command_servo || near(stopped.servo_deg, 0))
           && "L06 deadman servo 0 hoac khong lenh servo");

    const double speeds[] = {0, 2.5, 5};
    for (double speed : speeds) {
        options.speed_cms = speed;
        SteerGate speed_gate(options);
        const auto output = drive(speed_gate, 0);
        assert(near(output.motor_cms, speed) && output.motor_cms <= 5
               && "L07 ton trong toc do dat va tran 5");
    }
}

// L08/M3: cả hai dấu, ba trần; rate đủ cao để không che lỗi kẹp.
static void clamp_case() {
    const double limits[] = {0, 10, 40};
    const double signs[] = {-1, 1};
    for (double limit : limits) {
        for (double sign : signs) {
            Options options = assist_options();
            options.max_steer_deg = limit;
            options.max_steer_rate_deg_s = 1000;
            SteerGate gate(options);
            drive(gate, 0);
            const auto output = drive(gate, 1000, sign * 100);
            assert(near(output.servo_deg, sign * limit)
                   && "L08 M3 kep dung tran ca hai chieu");
        }
    }
}

// L09–L10: thời gian không đều, đảo chiều và dt=0.
static void slew_cases() {
    SteerGate gate(assist_options());
    drive(gate, 0);
    auto before = drive(gate, 1000);
    const long long times[] = {1100, 1350, 1400};
    const double targets[] = {35, 35, -35};
    const double deltas[] = {2, 5, 1};
    for (int i = 0; i < 3; ++i) {
        const auto after = drive(gate, times[i], targets[i]);
        const double change = after.servo_deg - before.servo_deg;
        assert(magnitude(change) <= deltas[i] + 1e-8
               && "L09 gioi han theo dt thuc");
        assert((i < 2 ? change > 0 : change < 0)
               && "L09 co tien trien dung chieu");
        before = after;
    }
    SteerGate zero_dt_gate(assist_options());
    const auto first = drive(zero_dt_gate, 0);
    const auto second = drive(zero_dt_gate, 0, 35);
    assert(near(second.servo_deg, first.servo_deg)
           && "L10 dt bang 0 khong doi goc");
}

// L11–L13/M2: đúng N khung liên tiếp, bằng ngưỡng được nhận.
static void lane_loss_cases() {
    SteerGate gate(assist_options());
    drive(gate, 0);
    drive(gate, 1000, 10);
    for (int n = 1; n <= 3; ++n) {
        TickInput input = tick(1000 + n);
        input.advice.conf = 0.59;
        const auto output = gate.step(input);
        if (n < 3) {
            assert(near(output.motor_cms, 5)
                   && output.stop_reason != StopReason::lane_lost
                   && "L11 chua du N thi chua dung mat lan");
        } else {
            assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
                   && output.stop_reason == StopReason::lane_lost
                   && output.need_new_press && "L11 M2 du N phai dung");
        }
    }

    SteerGate reset_gate(assist_options());
    drive(reset_gate, 0);
    for (int n = 1; n <= 6; ++n) {
        TickInput input = tick(n * 10);
        input.advice.conf = n == 3 ? 0.6 : 0.59;
        const auto output = reset_gate.step(input);
        if (n < 6) {
            assert(near(output.motor_cms, 5)
                   && output.stop_reason != StopReason::lane_lost
                   && "L12 khung bang nguong xoa chuoi mat lan");
        } else {
            assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
                   && output.stop_reason == StopReason::lane_lost
                   && output.need_new_press && "L12 chuoi moi du N moi dung");
        }
    }

    SteerGate absent_gate(assist_options());
    drive(absent_gate, 0);
    for (int n = 1; n <= 3; ++n) {
        TickInput input = tick(n * 10);
        input.advice.has_lane = false;
        const auto output = absent_gate.step(input);
        if (n == 3) {
            assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
                   && output.stop_reason == StopReason::lane_lost
                   && output.need_new_press && "L13 khong lan du conf cao");
        }
    }
}

// L14–L15: tuổi ảnh >500, dừng không phải đi qua rate góc thường.
static void stale_and_stop_slew_cases() {
    SteerGate gate(assist_options());
    drive(gate, 0);
    drive(gate, 1000, 10);
    const long long ages[] = {499, 500, 501};
    for (int n = 0; n < 3; ++n) {
        TickInput input = tick(1001 + n);
        input.frame_age_ms = ages[n];
        input.advice.angle_deg = 10;
        const auto output = gate.step(input);
        if (n < 2) {
            assert(near(output.motor_cms, 5)
                   && output.stop_reason != StopReason::frame_stale
                   && "L14 tuoi anh 499 va 500 con hop le");
        } else {
            assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
                   && output.stop_reason == StopReason::frame_stale
                   && output.need_new_press && "L14 tuoi anh 501 phai dung");
        }
    }
    for (int stale = 0; stale <= 1; ++stale) {
        Options options = assist_options();
        options.lost_lane_frames = 1;
        SteerGate stop_gate(options);
        drive(stop_gate, 0);
        const auto moving = drive(stop_gate, 1000, 10);
        assert(magnitude(moving.servo_deg) > 1 && "L15 thiet lap goc khac 0");
        TickInput input = tick(1001);
        input.frame_age_ms = stale ? 501 : 0;
        input.advice.conf = stale ? 0.9 : 0.1;
        const auto output = stop_gate.step(input);
        assert(near(output.motor_cms, 0) && near(output.servo_deg, 0)
               && "L15 dung loi bo qua rate de ve 0 ngay");
    }
}

// L16/M4: Stop giữ góc; Emergency trả 0; cả hai thắng lỗi cùng nhịp.
static void stop_priority_case() {
    const StopKey stops[] = {StopKey::Stop, StopKey::Emergency};
    for (StopKey stop : stops) {
        for (int fault = 0; fault < 4; ++fault) {
            Options options = assist_options();
            options.lost_lane_frames = 1;
            SteerGate gate(options);
            drive(gate, 0);
            const auto moving = drive(gate, 1000, 10);
            assert(magnitude(moving.servo_deg) > 1 && "L16 dang co goc");
            TickInput input = tick(1001);
            input.stop_key = stop;
            input.advice.angle_deg = -35;
            input.frame_age_ms = (fault == 1 || fault == 3) ? 501 : 0;
            input.advice.conf = fault >= 2 ? 0.1 : 0.9;
            const auto output = gate.step(input);
            assert(near(output.motor_cms, 0) && output.need_new_press
                   && output.stop_reason == StopReason::stop_key
                   && "L16 M4 stop thang W va loi cung nhip");
            if (stop == StopKey::Stop) {
                assert((!output.command_servo || near(output.servo_deg, moving.servo_deg))
                       && "L16 S Space giu goc");
            } else {
                assert(output.command_servo && near(output.servo_deg, 0)
                       && "L16 X Esc CtrlC tra servo 0");
            }
            const auto held = drive(gate, 1002, 35);
            assert(near(held.motor_cms, 0) && held.need_new_press
                   && "L16 W ton khong duoc chay lai");
        }
    }
}

// Dùng cùng bốn nguyên nhân để kiểm L17 và L18, không đọc trạng thái nội bộ.
static long long trigger_stop(SteerGate& gate, int cause) {
    drive(gate, 0);
    drive(gate, 1000, 10);
    Decision output{};
    const int count = cause == 0 ? 3 : 1;
    for (int n = 1; n <= count; ++n) {
        TickInput input = tick(1000 + n);
        input.advice.conf = cause == 0 ? 0.1 : 0.9;
        input.frame_age_ms = cause == 1 ? 501 : 0;
        input.stop_key = cause == 2 ? StopKey::Stop : StopKey::None;
        input.w_deadman_valid = cause != 3;
        output = gate.step(input);
    }
    assert(near(output.motor_cms, 0) && output.need_new_press
           && "L17/L18 thiet lap dung va chot");
    return 1000 + count;
}

// L17: có ảnh tốt hoặc qua nhiều hạn giữ vẫn không bỏ chốt W cũ.
static void latch_case() {
    for (int cause = 0; cause < 4; ++cause) {
        SteerGate gate(assist_options());
        const long long stopped_at = trigger_stop(gate, cause);
        for (long long elapsed = 1; elapsed <= 3001; elapsed += 100) {
            const auto output = drive(gate, stopped_at + elapsed, 20);
            assert(near(output.motor_cms, 0) && output.need_new_press
                   && "L17 true lien tuc khong mo chot");
        }
    }
}

// L18: giả định false biểu diễn khoảng im ở giao diện thuần, xem đầu bảng ca.
static void new_press_case() {
    const int holds[] = {700, 800, 2000};
    for (int hold : holds) {
        for (int cause = 0; cause < 4; ++cause) {
            Options options = assist_options();
            options.hold_timeout_ms = hold;
            SteerGate gate(options);
            const long long old_press_at = trigger_stop(gate, cause) + 1;
            const auto old_press = drive(gate, old_press_at);
            assert(near(old_press.motor_cms, 0) && "L18 W cu bi chan");
            TickInput quiet = tick(old_press_at);
            quiet.w_deadman_valid = false;
            gate.step(quiet);
            quiet.now_ms = old_press_at + hold - 1;
            gate.step(quiet);
            const auto early = drive(gate, quiet.now_ms);
            assert(near(early.motor_cms, 0) && early.need_new_press
                   && "L18 thieu 1ms chua du khoang im");

            // W vừa bị chặn cũng là lần lặp cũ cuối: tính lại khoảng im.
            const long long retry_at = quiet.now_ms;
            gate.step(quiet);
            quiet.now_ms = retry_at + hold;
            const auto still_quiet = gate.step(quiet);
            assert(near(still_quiet.motor_cms, 0)
                   && "L18 du khoang im nhung chua W van dung");
            const auto fresh = drive(gate, quiet.now_ms + 1);
            assert(near(fresh.motor_cms, 5) && !fresh.need_new_press
                   && "L18 du khoang im va W moi thi chay");
        }
    }
}

// L19–L22: chỉ quyết định nhận/bỏ; file/log thật thuộc Linux.
static void snapshot_cases() {
    SnapLimiter limiter(1024);
    long long accepted_times[26] = {};
    int count = 0;
    for (long long now = 0; now <= 2500; now += 100) {
        const auto output = limiter.request(now, false, 2048);
        if (now == 0) {
            assert(output.status == SnapStatus::accepted && "L19 nhan G dau");
        }
        if (output.status == SnapStatus::accepted) {
            accepted_times[count++] = now;
            if (count >= 3) {
                assert(now - accepted_times[count - 3] >= 1000
                       && "L19 toi da 2 snap trong 1 giay");
            }
        } else {
            assert(output.status == SnapStatus::rate_limited
                   && "L19 chi duoc bo vi han muc");
        }
    }

    SnapLimiter recovering(1024);
    const long long times[] = {0, 1001, 2002};
    for (long long now : times) {
        assert(recovering.request(now, false, 2048).status == SnapStatus::accepted
               && "L20 qua cua so phai nhan lai");
    }

    SnapLimiter busy_limiter(1024);
    const auto busy = busy_limiter.request(0, true, 2048);
    assert(busy.status == SnapStatus::busy && busy.log_busy
           && "L21 writer ban bo va yeu cau log");
    assert(busy_limiter.request(0, false, 2048).status == SnapStatus::accepted
           && "L21 lan bo khong an quota");

    SnapLimiter disk_limiter(1024);
    const auto low = disk_limiter.request(0, false, 1023);
    assert(low.status == SnapStatus::disk_low && low.disk_warning
           && "L22 duoi min khong luu va canh bao");
    assert(disk_limiter.request(0, false, 1024).status == SnapStatus::accepted
           && "L22 bang min duoc luu sau khi dia phuc hoi");
}

// L23/M5: phím G thô đi qua KeyDeadman, không cấp sẵn bool giả đúng.
static void g_deadman_case() {
    const long long t0 = 1000;
    KeyDeadman keys(800);
    KeyDeadman without_g(800);
    SteerGate gate(assist_options());
    keys.onKey(t0, 'W');
    without_g.onKey(t0, 'W');
    TickInput input = tick(t0);
    input.w_deadman_valid = keys.wValid(t0);
    assert(input.w_deadman_valid && "L23 W khoi dong han giu");
    assert(near(gate.step(input).motor_cms, 5) && "L23 W cho chay");
    const long long offsets[] = {400, 700, 799, 801, 900, 1701};
    for (long long elapsed : offsets) {
        const long long now = t0 + elapsed;
        if (elapsed == 400 || elapsed == 700 || elapsed == 900) {
            keys.onKey(now, 'G');
        }
        const bool valid = keys.wValid(now);
        assert(valid == without_g.wValid(now) && "L23 G khong doi han W");
        assert(valid == (elapsed < 800) && "L23 M5 qua 800ms W het han");
        input = tick(now);
        input.w_deadman_valid = valid;
        const auto output = gate.step(input);
        if (elapsed > 800) {
            assert(near(output.motor_cms, 0) && output.need_new_press
                   && "L23 G khong gia han hay tu chay lai");
        } else {
            assert(near(output.motor_cms, 5) && "L23 G khong dung W con han");
        }
    }
}

// L24: cùng now_ms là một lô; G không tăng hoặc xóa số phím chuyển động.
static void g_burst_case() {
    const char* batches[] = {"WGGG", "WADCG", "WADC"};
    const long long gaps[] = {400, 401};
    const long long t0 = 1000;
    for (long long gap : gaps) {
        for (int n = 0; n < 3; ++n) {
            KeyDeadman keys(800);
            SteerGate gate(assist_options());
            keys.onKey(t0, 'W');
            TickInput input = tick(t0);
            input.w_deadman_valid = keys.wValid(t0);
            assert(near(gate.step(input).motor_cms, 5) && "L24 dang chay truoc im");
            const long long now = t0 + gap;
            keys.onKey(now - 1, 'G'); // G cũng không được cắt khoảng im W/A/D/C.
            for (const char* key = batches[n]; *key; ++key) {
                keys.onKey(now, *key);
            }
            input = tick(now);
            input.w_deadman_valid = keys.wValid(now);
            assert(input.w_deadman_valid == (n == 0)
                   && "L24 WGGG khong burst; WADCG WADC thanh burst");
            const auto output = gate.step(input);
            if (n == 0) {
                assert(near(output.motor_cms, 5) && !output.need_new_press
                       && "L24 G khong tinh trong lo");
            } else {
                assert(near(output.motor_cms, 0) && output.need_new_press
                       && "L24 burst bo chuyen dong va chot");
                keys.onKey(now + 1, 'W');
                assert(!keys.wValid(now + 1) && "L24 W lap khong mo chot burst");
            }
        }
    }
}

// L25: giá trị không hữu hạn không được lọt qua so sánh miền thông thường.
static void invalid_options_cases() {
    const double invalid[] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()};
    for (double value : invalid) {
        Options options = assist_options();
        options.speed_cms = value;
        assert(!validateOptions(options) && "L25 speed NaN inf bi bac");
        options = assist_options();
        options.max_steer_deg = value;
        assert(!validateOptions(options) && "L25 max steer NaN inf bi bac");
    }
    const int counts[] = {-1, 0, 1};
    for (int count : counts) {
        Options options = assist_options();
        options.lost_lane_frames = count;
        assert(validateOptions(options) == (count > 0) && "L25 N phai duong");
    }
}

// L26: NaN/tuổi ảnh âm không được phát lệnh servo, kể cả servo 0.
static void invalid_tick_cases() {
    for (int fault = 0; fault < 3; ++fault) {
        SteerGate gate(assist_options());
        drive(gate, 0);
        const auto moving = drive(gate, 1000, 10);
        assert(magnitude(moving.servo_deg) > 1 && "L26 dang chay co goc");
        TickInput input = tick(1001);
        if (fault == 0) {
            input.advice.angle_deg = std::numeric_limits<double>::quiet_NaN();
        } else if (fault == 1) {
            input.advice.conf = std::numeric_limits<double>::quiet_NaN();
        } else {
            input.frame_age_ms = -1;
        }
        const auto output = gate.step(input);
        assert(near(output.motor_cms, 0) && !output.command_servo
               && "L26 dau vao loi motor 0 va khong lenh servo");
    }
}

// L27: hai điều kiện cùng tới hạn, frame_stale phải thắng lane_lost.
static void stale_over_lane_case() {
    SteerGate gate(assist_options());
    drive(gate, 0);
    const auto moving = drive(gate, 1000, 10);
    assert(magnitude(moving.servo_deg) > 1 && "L27 dang co goc");
    for (int n = 1; n <= 3; ++n) {
        TickInput input = tick(1000 + n);
        input.advice.has_lane = false;
        input.advice.conf = 0.1;
        input.frame_age_ms = n == 3 ? 501 : 0;
        const auto output = gate.step(input);
        if (n < 3) {
            assert(near(output.motor_cms, 5) && "L27 chua du N chua dung");
        } else {
            assert(output.stop_reason == StopReason::frame_stale
                   && near(output.motor_cms, 0) && output.command_servo
                   && near(output.servo_deg, 0) && output.need_new_press
                   && "L27 frame stale thang lane lost cung nhip");
        }
    }
}

int main() {
    options_cases();
    suggestion_case();
    deadman_and_speed_cases();
    clamp_case();
    slew_cases();
    lane_loss_cases();
    stale_and_stop_slew_cases();
    stop_priority_case();
    latch_case();
    new_press_case();
    snapshot_cases();
    g_deadman_case();
    g_burst_case();
    invalid_options_cases();
    invalid_tick_cases();
    stale_over_lane_case();
    return 0;
}
