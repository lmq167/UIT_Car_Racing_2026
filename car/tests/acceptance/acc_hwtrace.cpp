// Test chấp nhận – kiểm tra LỆNH PHẦN CỨNG GIẢ qua vết UCR_FAKE_HW_TRACE, chạy lớp App thật ở
// UCR_FAKE_HW=ON với bộ điều khiển (IController) do test tiêm bằng App::SetController (giao diện công khai).
//
// Yêu cầu gốc (chỉ dùng văn bản yêu cầu/tài liệu, không dựa vào cách cài đặt):
//  - Quy tắc an toàn khi chạy xe: (a) dừng motor + trả servo về 0 (= về trim) khi thoát/lỗi/mất ảnh > 0,5 s;
//    (b) giới hạn tốc độ tối đa trong config; (c) ToF phía trước phanh mặc định bật. "Chuỗi khởi tạo motor ... chỉ chạy
//    đúng MỘT lần lúc khởi động chương trình chính".
//  - Yêu cầu kiểm thử: trim + đảo chiều servo áp dụng đúng; phanh ToF (tốc độ ra = 0 trong <= 1 chu kỳ điều khiển; đang lùi
//    không chặn; ToF -1 xử lý theo tài liệu); mất ảnh; lệnh cuối cùng gửi phần cứng là tốc độ 0, servo về trim;
//    chuỗi init đúng 1 lần; lệnh tốc độ chỉ gửi khi đổi hoặc theo chu kỳ giữ kết nối trong config.
//  - docs/ucr_car.md: bảng khoá (servo_trim_deg: "dừng xe trả về góc này, không đảo dấu trim"; servo_invert: "đảo chiều góc
//    tương đối tâm"; motor_keepalive_ms; tof_brake_mm "so sánh nhỏ hơn"; tof_timeout_ms "mất/không hợp lệ/quá cũ chặn tiến";
//    image_timeout_ms), mục "Luồng, an toàn và dừng" ("Chiều servo được áp dụng trước cộng trim"; "Chỉ gửi khi tốc độ thay đổi
//    hoặc đến chu kỳ duy trì kết nối"), mục "Điểm cắm kiểm thử bản giả lập" (định dạng vết, ToF giả, khoảng ngắt camera, lỗi giả).
//
// Mỗi kịch bản chạy trong MỘT tiến trình riêng (biến môi trường UCR_FAKE_* được đọc một lần). Dùng: acc_hwtrace <kịch_bản>
// Mã thoát 77 = CHƯA KIỂM ĐƯỢC (điều kiện tiền đề không xảy ra, ví dụ xe chưa kịp chạy trước mốc thử).
//
// Các ngưỡng thời gian tự đặt (ghi rõ để xem xét, đã chọn có biên độ cho máy chậm):
//  - Phanh ToF: độ trễ từ lúc kịch bản ToF đổi đến lúc lệnh tốc độ 0 xuất hiện trong vết <= chu kỳ cảm biến + chu kỳ điều khiển
//    + 100 ms. Lý do: ToF giả "được công bố lại mỗi vòng cảm biến" (tối đa 1 chu kỳ cảm biến chờ) rồi lớp an toàn phải ra
//    lệnh 0 trong 1 chu kỳ điều khiển; 100 ms là dư cho lịch hệ điều hành/ghi vết. (50 Hz: 140 ms; 10 Hz: 300 ms.)
//  - Mất ảnh: lệnh phải là 0/trim kể từ mốc "bắt đầu ngắt + image_timeout_ms + 1 chu kỳ điều khiển + 100 ms".
//  - Lỗi trong luồng (ném ngoại lệ): App tự dừng (IsRunning()==false) trong <= 2 s (bộ điều khiển ném: <= 1,5 s) sau lỗi.
//  - Gửi lệnh tốc độ khi đổi giá trị: <= 150 ms; chu kỳ giữ kết nối: khoảng cách giữa hai lệnh lặp trong [0,8K, K+150] ms.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include "acc_common.h"
#include "acc_trace.h"
#include "acc_seed_gate.h"
#include "app/App.h"
#include "app/Backend.h"
#include "app/IController.h"
#include "common/Config.h"
#ifndef UCR_FAKE_HW
#error "acc_hwtrace chi dung duoc voi UCR_FAKE_HW=ON"
#endif
#include "app/FakeHardware.h"  // chỉ để lấy đồng hồ mili-giây chung với vết (app::fake::Millis)

namespace {

using acc::Trace;
using acc::TraceLine;

long long Now() { return app::fake::Millis(); }

// Bộ điều khiển tiêm vào: fn(thời gian từ lần gọi đầu [ms], thời gian tuyệt đối trong tiến trình [ms]).
class FnController : public app::IController {
public:
    using Fn = std::function<common::ControlCommand(long long, long long)>;
    explicit FnController(Fn fn) : fn_(std::move(fn)) {}
    common::ControlCommand step(const common::Observation &) override {
        const long long now = Now();
        if (first_ < 0) first_ = now;
        return fn_(now - first_, now);
    }
private:
    Fn fn_;
    long long first_ = -1;
};

struct Ctx {
    std::string root, logdir, trace, cfg;
    common::Config c;
};

Ctx Prepare() {
    Ctx x;
    x.root = acc::MakeTempDir("acc_hw_");
    if (x.root.empty()) { std::perror("mkdtemp"); std::exit(2); }
    x.logdir = x.root + "/runs";
    mkdir(x.logdir.c_str(), 0755);
    x.trace = x.root + "/trace.csv";
    x.cfg = x.root + "/config.yaml";
    acc::WriteFile(x.cfg, "%YAML:1.0\n---\nlog_dir: \"" + x.logdir + "\"\nenable_display: 0\nenable_oled: 0\ntof_mapping_confirmed: 1\n");
    setenv("UCR_FAKE_HW_TRACE", x.trace.c_str(), 1);
    x.c.log_dir = x.logdir;
    x.c.enable_display = 0;
    x.c.enable_oled = 0;
    x.c.max_speed_cms = 12.5f;
    x.c.max_steering_deg = 10;
    x.c.fake_source = "";
    x.c.image_every_n = 1000000;  // gần như không lưu JPEG: giữ thư mục tạm nhỏ
    return x;
}

struct Outcome {
    bool self_stopped = false, failed = false, waited = false;
    long wait_ms = -1;
    long long begin_ms = 0, noticed_ms = 0, stop_req_ms = 0;
    Trace after_wait;
};

// Chạy App ở chế độ mode trong run_ms (hoặc đến khi App tự dừng), rồi RequestStop + Wait (trừ khi destroy_only).
Outcome RunApp(Ctx &x, const std::string &mode, std::unique_ptr<app::IController> ctl, long long run_ms, bool destroy_only = false) {
    Outcome o;
    if(mode=="auto") { x.c.tof_mapping_confirmed=1; acc::SeedGate(x.c,x.cfg); }
    auto fut = std::async(std::launch::async, [&] {
        auto a = std::make_unique<app::App>(x.c, mode, x.cfg);
        a->Init();
        if (ctl) a->SetController(std::move(ctl));
        a->Start();
        o.begin_ms = Now();
        while (Now() - o.begin_ms < run_ms) {
            if (!a->IsRunning()) break;
            acc::SleepMs(10);
        }
        o.noticed_ms = Now();
        o.self_stopped = !a->IsRunning();
        o.failed = a->Failed();
        o.stop_req_ms = Now();
        if (!destroy_only) {
            const auto t1 = std::chrono::steady_clock::now();
            a->RequestStop();
            a->Wait();
            o.wait_ms = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count());
            o.waited = true;
            o.after_wait = acc::ReadTrace(x.trace);
        }
        a.reset();  // huỷ App (đường huỷ đối tượng cũng được phép gửi lệnh dừng)
    });
    if (fut.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        std::fprintf(stderr, "[HONG] khong ket thuc sau 60 s (treo)\n");
        std::fflush(stderr);
        _exit(1);
    }
    try { fut.get(); } catch (const std::exception &ex) {
        ACC_CHECK(false, std::string("ngoai le thoat ra khoi App: ") + ex.what());
    }
    return o;
}

// Kiểm tra chung mọi kịch bản: vết đúng định dạng; không lệnh tốc độ nào vượt max_speed_cms; chuỗi init đúng 1 lần.
Trace CommonChecks(const Ctx &x, const Outcome &o, bool expect_init = true) {
    int bad = 0;
    const Trace t = acc::ReadTrace(x.trace, &bad);
    ACC_CHECK(bad == 0, "vet phan cung co " + std::to_string(bad) + " dong sai dinh dang");
    ACC_CHECK(!t.empty(), "vet phan cung rong (UCR_FAKE_HW_TRACE khong duoc ghi)");
    for (const auto &l : t)
        if (l.kind == "motor_speed")
            ACC_CHECK(std::isfinite(l.num) && std::fabs(l.num) <= x.c.max_speed_cms + 1e-3,
                      "lenh toc do gui phan cung " + l.value + " vuot max_speed_cms=" + std::to_string(x.c.max_speed_cms));
    if (expect_init) {
        ACC_CHECK(acc::Count(t, "motor_init") == 1, "motor_init phai dung 1 lan, thay " + std::to_string(acc::Count(t, "motor_init")));
        size_t save = 0;
        for (const auto &l : acc::Of(t, "motor_cfg")) if (l.value.find("$101=1") != std::string::npos) ++save;
        ACC_CHECK(save == 1, "lenh luu ROM/reset ($101=1) phai dung 1 lan, thay " + std::to_string(save));
    }
    if (o.waited) ACC_CHECK(o.wait_ms >= 0 && o.wait_ms <= 3000, "RequestStop->Wait phai xong <= 3 s, mat " + std::to_string(o.wait_ms) + " ms");
    return t;
}

// Lệnh cuối cùng gửi phần cứng: tốc độ 0, servo về trim.
void CheckTail(const Trace &t, double trim, const std::string &where) {
    double v = 1e9;
    ACC_CHECK(acc::LastNum(t, "motor_speed", v), where + ": khong co lenh motor_speed nao");
    if (v < 1e8) ACC_CHECK(acc::Near(v, 0.0), where + ": lenh motor_speed cuoi cung phai la 0, thay " + std::to_string(v));
    v = 1e9;
    ACC_CHECK(acc::LastNum(t, "servo_angle", v), where + ": khong co lenh servo_angle nao");
    if (v < 1e8) ACC_CHECK(acc::Near(v, trim), where + ": lenh servo_angle cuoi cung phai la trim=" + std::to_string(trim) + ", thay " + std::to_string(v));
}

// Giá trị của loại kind tại thời điểm tmoc: dòng cuối có ms <= tmoc. false nếu chưa có dòng nào.
bool StateAt(const Trace &t, const std::string &kind, long long tmoc, double &v) {
    bool f = false;
    for (const auto &l : t) if (l.kind == kind && l.ms <= tmoc) { v = l.num; f = true; }
    return f;
}

int Inconclusive(const std::string &why) {
    std::fprintf(stderr, "CHUA KIEM DUOC: %s\n", why.c_str());
    return 77;
}

// ---------------------------------------------------------------------------------------------------------------------
// Ca 1: trim + đảo chiều servo áp dụng đúng lên lệnh servo (góc gửi servo = [đảo?](góc tương đối) + trim, chặn ±60).
int ServoScenario(const std::string &name) {
    Ctx x = Prepare();
    int trim = 7, inv = 0;
    std::vector<int> steer = {10, -10, 4};
    std::vector<double> expect = {17, -3, 11};
    if (name == "servo_invert") { inv = 1; expect = {-3, 17, 3}; }
    else if (name == "servo_edge_pos") { trim = 40; x.c.max_steering_deg = 20; steer = {1000, -1000}; expect = {60, 20}; }
    else if (name == "servo_edge_neg") { trim = -40; x.c.max_steering_deg = 20; steer = {-1000, 1000}; expect = {-60, -20}; }
    x.c.servo_trim_deg = trim;
    x.c.servo_invert = inv;
    const int phase_ms = 450;
    auto ctl = std::make_unique<FnController>([=](long long rel, long long) {
        size_t i = std::min<size_t>(static_cast<size_t>(rel / phase_ms), steer.size() - 1);
        return common::ControlCommand{6.0f, steer[i]};
    });
    const auto o = RunApp(x, "auto", std::move(ctl), static_cast<long long>(steer.size()) * phase_ms + 700);
    const Trace t = CommonChecks(x, o);
    if (!acc::Moved(t)) return Inconclusive("xe khong di chuyen (bo dieu khien khong duoc goi?)");

    std::vector<double> seq;
    for (const auto &l : acc::Of(t, "servo_angle")) {
        ACC_CHECK(std::fabs(l.num) <= 60.0 + 1e-3, "servo_angle " + l.value + " vuot +-60");
        if (acc::Near(l.num, trim)) continue;
        if (seq.empty() || !acc::Near(seq.back(), l.num)) seq.push_back(l.num);
    }
    std::string got;
    for (double d : seq) got += std::to_string(d) + " ";
    for (double d : seq) {
        bool ok = false;
        for (double e : expect) ok = ok || acc::Near(d, e);
        ACC_CHECK(ok, "servo_angle " + std::to_string(d) + " khong nam trong tap mong doi (trim=" + std::to_string(trim) + ", invert=" +
                      std::to_string(inv) + "); chuoi thay: " + got);
    }
    size_t k = 0;
    for (double d : seq) if (k < expect.size() && acc::Near(d, expect[k])) ++k;
    ACC_CHECK(k == expect.size(), "chuoi goc servo (bo gia tri = trim) phai chua theo thu tu cac gia tri mong doi; thay: " + got);
    CheckTail(o.after_wait, trim, "sau Wait");
    CheckTail(acc::ReadTrace(x.trace), trim, "sau khi huy App");

    if (name == "servo_trim" || name == "servo_invert") {
        // cmd_steer trong log.csv là góc tương đối tâm, CHƯA cộng trim: độ lớn chỉ thuộc {0, 10, 4}.
        const std::string run = acc::FindRunDir(x.logdir, "_auto");
        const acc::Csv csv = acc::ReadCsv(run + "/log.csv");
        for (size_t i = 0; i < csv.rows.size(); ++i) {
            const double a = std::fabs(csv.Num(i, "cmd_steer"));
            ACC_CHECK(acc::Near(a, 0) || acc::Near(a, 10) || acc::Near(a, 4), "cmd_steer trong log.csv phai la goc tuong doi tam (khong cong trim), thay " + std::to_string(csv.Num(i, "cmd_steer")));
        }
    }
    return acc::Finish(("acc_hwtrace " + name).c_str());
}

// ---------------------------------------------------------------------------------------------------------------------
// Ca 4: lệnh cuối cùng gửi phần cứng là tốc độ 0 + servo về trim sau: thoát bình thường, hủy App khi đang chạy,
// ngoại lệ trong bộ điều khiển, ngoại lệ trong luồng Actuator / Sensor (UCR_FAKE_FAULT).
int TailScenario(const std::string &name) {
    Ctx x = Prepare();
    const int trim = 7;
    x.c.servo_trim_deg = trim;
    std::atomic<long long> thrown_ms{-1};
    long long fault_ms = -1;
    std::function<common::ControlCommand(long long, long long)> fn = [](long long, long long) { return common::ControlCommand{8.0f, 10}; };
    long long run_ms = 1500;
    bool destroy_only = false;
    if (name == "tail_destroy") destroy_only = true;
    else if (name == "tail_ctl_throw") {
        run_ms = 5000;
        fn = [&thrown_ms](long long rel, long long abs) -> common::ControlCommand {
            if (rel > 900) { long long e = -1; thrown_ms.compare_exchange_strong(e, abs); throw std::runtime_error("loi gia lap trong bo dieu khien"); }
            return {8.0f, 10};
        };
    } else if (name == "tail_actuator_throw" || name == "tail_sensor_throw") {
        fault_ms = 2200;
        run_ms = 6000;
        setenv("UCR_FAKE_FAULT", (std::string(name == "tail_actuator_throw" ? "actuator_throw" : "sensor_throw") + "@2200").c_str(), 1);
    }
    auto ctl = std::make_unique<FnController>(fn);
    const auto o = RunApp(x, "auto", std::move(ctl), run_ms, destroy_only);
    const Trace t = acc::ReadTrace(x.trace);
    if (!acc::Moved(t)) return Inconclusive("xe khong di chuyen truoc khi dung (bo dieu khien khong duoc goi / loi xay ra truoc khi chay)");
    const Trace all = CommonChecks(x, o);

    if (name == "tail_ctl_throw") {
        ACC_CHECK(thrown_ms.load() > 0, "bo dieu khien phai ne loi (tien de)");
        ACC_CHECK(o.self_stopped, "ngoai le tu bo dieu khien phai lam App tu dung");
        ACC_CHECK(o.failed, "ngoai le tu bo dieu khien phai danh dau Failed()");
        if (thrown_ms.load() > 0) ACC_CHECK(o.noticed_ms - thrown_ms.load() <= 1500, "App phai tu dung <= 1.5 s sau ngoai le, mat " + std::to_string(o.noticed_ms - thrown_ms.load()) + " ms");
    } else if (fault_ms > 0) {
        ACC_CHECK(o.self_stopped, "loi trong luong phai lam App tu yeu cau dung (IsRunning()==false) trong <= 2 s sau loi");
        ACC_CHECK(o.failed, "loi trong luong phai danh dau Failed()==true");
        if (o.self_stopped) ACC_CHECK(o.noticed_ms - fault_ms <= 2000, "App phai tu dung <= 2 s sau loi gia @" + std::to_string(fault_ms) + " ms, nhan thay luc " + std::to_string(o.noticed_ms) + " ms");
        // Lệnh 0 phải xuất hiện sớm: trạng thái tại fault_ms+2000 đã là 0 / trim.
        double v = 1e9;
        if (StateAt(all, "motor_speed", fault_ms + 2000, v)) ACC_CHECK(acc::Near(v, 0.0), "toc do gui phan cung tai fault+2 s phai la 0, thay " + std::to_string(v));
        if (StateAt(all, "servo_angle", fault_ms + 2000, v)) ACC_CHECK(acc::Near(v, trim), "servo gui phan cung tai fault+2 s phai la trim, thay " + std::to_string(v));
    }
    if (o.waited) CheckTail(o.after_wait, trim, "sau Wait");
    CheckTail(all, trim, "sau khi huy App");
    return acc::Finish(("acc_hwtrace " + name).c_str());
}

// ---------------------------------------------------------------------------------------------------------------------
// Ca 7: lệnh tốc độ chỉ gửi khi đổi giá trị hoặc theo chu kỳ motor_keepalive_ms; đổi góc riêng không gửi lại tốc độ.
int KeepaliveScenario(const std::string &name) {
    Ctx x = Prepare();
    const int K = name == "keepalive_100" ? 100 : 200;
    x.c.motor_keepalive_ms = K;
    const bool toggle = name == "keepalive_steer";
    auto ctl = std::make_unique<FnController>([=](long long rel, long long) {
        const int steer = toggle ? (((rel / 100) % 2) ? 3 : -3) : 0;
        return common::ControlCommand{8.0f, steer};
    });
    const auto o = RunApp(x, "auto", std::move(ctl), 2600);
    const Trace t = CommonChecks(x, o);
    Trace v8;
    for (const auto &l : acc::Of(t, "motor_speed")) if (acc::Near(l.num, 8.0) && l.ms < o.stop_req_ms) v8.push_back(l);
    if (v8.size() < 2) return Inconclusive("xe khong di chuyen / khong co lenh lap lai nao (chi " + std::to_string(v8.size()) + " dong toc do 8)");
    const double steady = static_cast<double>(o.stop_req_ms - v8.front().ms);
    const double expected = steady / K;
    ACC_CHECK(v8.size() >= 0.6 * expected, "phai gui lai toc do theo chu ky giu ket noi K=" + std::to_string(K) + " ms: ky vong ~" +
              std::to_string(expected) + " dong, thay " + std::to_string(v8.size()));
    ACC_CHECK(v8.size() <= 1.3 * expected + 2, "chi gui toc do khi doi hoac moi K=" + std::to_string(K) + " ms: ky vong ~" + std::to_string(expected) +
              " dong, thay " + std::to_string(v8.size()) + " (gui lap moi vong dieu khien?)");
    for (size_t i = 1; i < v8.size(); ++i) {
        const long long gap = v8[i].ms - v8[i - 1].ms;
        ACC_CHECK(gap >= 0.8 * K, "hai lenh toc do giong nhau cach nhau " + std::to_string(gap) + " ms < 0.8*K (K=" + std::to_string(K) + ")");
        ACC_CHECK(gap <= K + 150, "hai lenh toc do giong nhau cach nhau " + std::to_string(gap) + " ms > K+150 (K=" + std::to_string(K) + "): giu ket noi bi thieu");
    }
    return acc::Finish(("acc_hwtrace " + name).c_str());
}

// Tốc độ đổi (5 -> 9 -> 5) phải được gửi ngay (<= 150 ms), không đợi chu kỳ giữ kết nối.
int SpeedChangeScenario() {
    Ctx x = Prepare();
    x.c.motor_keepalive_ms = 200;
    std::atomic<long long> t9{-1}, t5b{-1};
    auto ctl = std::make_unique<FnController>([&](long long rel, long long abs) {
        float s = 5.0f;
        if (rel >= 1400) { s = 5.0f; long long e = -1; t5b.compare_exchange_strong(e, abs); }
        else if (rel >= 700) { s = 9.0f; long long e = -1; t9.compare_exchange_strong(e, abs); }
        return common::ControlCommand{s, 0};
    });
    const auto o = RunApp(x, "auto", std::move(ctl), 2300);
    const Trace t = CommonChecks(x, o);
    if (t9.load() < 0 || t5b.load() < 0) return Inconclusive("bo dieu khien khong duoc goi du lau");
    std::vector<double> seq;
    for (const auto &l : acc::Of(t, "motor_speed")) {
        if (l.ms >= o.stop_req_ms || std::fabs(l.num) < 0.5) continue;
        if (seq.empty() || !acc::Near(seq.back(), l.num)) seq.push_back(l.num);
    }
    ACC_CHECK(seq.size() == 3 && acc::Near(seq[0], 5) && acc::Near(seq[1], 9) && acc::Near(seq[2], 5),
              "chuoi toc do khac 0 gui phan cung phai la 5 -> 9 -> 5 (so dong: " + std::to_string(seq.size()) + ")");
    auto first_after = [&](double val, long long since, long long &out) {
        for (const auto &l : acc::Of(t, "motor_speed")) if (l.ms >= since - 5 && acc::Near(l.num, val)) { out = l.ms; return true; }
        return false;
    };
    long long at = 0;
    if (first_after(9.0, t9.load(), at)) ACC_CHECK(at - t9.load() <= 150, "toc do doi 5->9 phai duoc gui <= 150 ms, mat " + std::to_string(at - t9.load()) + " ms");
    else ACC_CHECK(false, "khong thay lenh toc do 9 sau khi bo dieu khien doi");
    if (first_after(5.0, t5b.load(), at)) ACC_CHECK(at - t5b.load() <= 150, "toc do doi 9->5 phai duoc gui <= 150 ms, mat " + std::to_string(at - t5b.load()) + " ms");
    else ACC_CHECK(false, "khong thay lenh toc do 5 sau khi bo dieu khien doi lai");
    return acc::Finish("acc_hwtrace speed_change");
}

// Ca 6: chuỗi khởi tạo motor đúng 1 lần dù luồng odometry/cảm biến chạy nhanh; đúng thứ tự BTC; không có tự dò PID.
int InitOnceScenario() {
    Ctx x = Prepare();
    x.c.odometry_hz = 100;
    x.c.sensor_hz = 100;
    auto ctl = std::make_unique<FnController>([](long long, long long) { return common::ControlCommand{4.0f, 0}; });
    const auto o = RunApp(x, "auto", std::move(ctl), 2500);
    const Trace t = CommonChecks(x, o);
    const Trace cfg = acc::Of(t, "motor_cfg");
    ACC_CHECK(cfg.size() == 7, "chuoi init motor gia lap gom 7 lenh (docs/ucr_car.md), thay " + std::to_string(cfg.size()));
    auto idx = [&](const char *needle) {
        int found = -1, n = 0;
        for (size_t i = 0; i < cfg.size(); ++i) if (cfg[i].value.find(needle) != std::string::npos) { ++n; if (found < 0) found = static_cast<int>(i); }
        return n == 1 ? found : -2;
    };
    const int a = idx("$008=0"), b = idx("$002=330"), s = idx("$101=1");
    ACC_CHECK(a >= 0 && b >= 0 && s >= 0, "moi lenh $008=0, $002=330, $101=1 phai xuat hien dung 1 lan");
    ACC_CHECK(a < s && b < s, "thu tu BTC: $008=0 va $002=330 truoc $101=1 (luu ROM/reset)");
    for (const auto &l : cfg) ACC_CHECK(l.value.size() < 4 || l.value.compare(l.value.size() - 4, 4, " O T") != 0, "khong duoc co lenh tu do PID (N1 O T): " + l.value);
    // Mọi lệnh cấu hình phải nằm TRƯỚC lệnh tốc độ đầu tiên khác 0 (không phát sinh lại khi đang chạy).
    long long first_move = -1, last_cfg = -1;
    for (const auto &l : t) {
        if (l.kind == "motor_speed" && std::fabs(l.num) > 0.5 && first_move < 0) first_move = l.ms;
        if (l.kind == "motor_cfg") last_cfg = l.ms;
    }
    if (first_move >= 0) ACC_CHECK(last_cfg <= first_move, "lenh cau hinh driver con xuat hien sau khi xe da chay");
    return acc::Finish("acc_hwtrace init_once");
}

// ---------------------------------------------------------------------------------------------------------------------
// Ca 2: phanh ToF trước đầu-cuối bằng UCR_FAKE_TOF_SCRIPT.
constexpr long long kT1 = 2500, kT2 = 3500;  // mốc đổi ToF (ms từ lúc khởi động tiến trình)

std::string TofRows(const std::string &rest) { return "0 2000 2000 2000 2000\n" + rest; }

int TofScenario(const std::string &name) {
    Ctx x = Prepare();
    float speed = 8.0f;
    std::string rows;
    long long run_ms = 4200;
    int hz = 50;
    const std::string T1 = std::to_string(kT1), T2 = std::to_string(kT2);
    enum Kind { Brake, Reverse, Rear, Order, Boundary, StartInvalid } kind = Brake;
    if (name == "tof_brake") rows = TofRows(T1 + " 100 2000 2000 2000\n");
    else if (name == "tof_invalid") rows = TofRows(T1 + " -1 2000 2000 2000\n");
    else if (name == "tof_brake_slow") { rows = TofRows(T1 + " 100 2000 2000 2000\n"); hz = 10; }
    else if (name == "tof_reverse") { kind = Reverse; speed = -8.0f; rows = TofRows(T1 + " 100 2000 2000 2000\n"); }
    else if (name == "tof_rear") { kind = Rear; rows = TofRows(T1 + " 2000 100 100 100\n"); }
    else if (name == "tof_order") {
        kind = Order; run_ms = 5200;
        x.c.tof_names = {"rear", "front", "left", "right"};
        rows = TofRows(T1 + " 100 2000 2000 2000\n" + T2 + " 2000 100 2000 2000\n");
    } else if (name == "tof_boundary") {
        kind = Boundary; run_ms = 5200;
        rows = TofRows(T1 + " 250 2000 2000 2000\n" + T2 + " 249 2000 2000 2000\n");
    } else if (name == "tof_start_invalid") { kind = StartInvalid; rows = "2600 2000 2000 2000 2000\n"; run_ms = 4500; }
    else return 2;
    x.c.control_hz = hz;
    x.c.sensor_hz = hz;
    const std::string script = x.root + "/tof_script.txt";
    acc::WriteFile(script, rows);
    setenv("UCR_FAKE_TOF_SCRIPT", script.c_str(), 1);
    auto ctl = std::make_unique<FnController>([=](long long, long long) { return common::ControlCommand{speed, 0}; });
    const auto o = RunApp(x, "auto", std::move(ctl), run_ms);
    const Trace t = CommonChecks(x, o);
    const Trace ms = acc::Of(t, "motor_speed");
    const long long bound = 1000 / hz + 1000 / hz + 100;

    // Dòng đầu tiên có tốc độ khác 0 ("đã chạy").
    const TraceLine *p = nullptr;
    for (const auto &l : ms) if (std::fabs(l.num) > 0.5) { p = &l; break; }
    // Trước mốc đổi ToF phải đã chạy được, nếu không thì bài thử vô nghĩa.
    const long long first_event = kind == StartInvalid ? 2600 : kT1;
    if (kind != StartInvalid) {
        if (!p || p->ms > first_event - 150) return Inconclusive("xe chua kip chay truoc moc doi ToF (dong chay dau tien luc " + (p ? std::to_string(p->ms) : std::string("khong co")) + " ms)");
    } else {
        if (o.begin_ms > 2000) return Inconclusive("khoi dong qua cham (" + std::to_string(o.begin_ms) + " ms) de kiem tra giai doan truoc dong ToF dau tien");
        for (const auto &l : ms) if (l.ms < 2600) ACC_CHECK(l.num <= 1e-3, "truoc dong ToF dau tien (2600 ms) moi slot khong hop le: khong duoc co lenh tien, thay " + l.value + " luc " + std::to_string(l.ms) + " ms");
        if (!p) return Inconclusive("xe khong chay duoc ca sau khi ToF hop le xuat hien");
        ACC_CHECK(p->ms >= 2600, "lenh tien dau tien luc " + std::to_string(p->ms) + " ms truoc khi ToF hop le xuat hien");
        ACC_CHECK(p->ms <= 2600 + bound + 400, "ToF da hop le tu 2600 ms nhung xe chi chay luc " + std::to_string(p->ms) + " ms");
        return acc::Finish("acc_hwtrace tof_start_invalid");
    }

    // Lần đầu về 0 sau khi chạy: z.
    auto first_zero_after = [&](long long since) -> const TraceLine * {
        for (const auto &l : ms) if (l.ms >= since && std::fabs(l.num) <= 1e-3) return &l;
        return nullptr;
    };
    auto no_zero_between = [&](long long a, long long b, const std::string &why) {
        for (const auto &l : ms) if (l.ms >= a && l.ms < b) ACC_CHECK(std::fabs(l.num) > 0.5, why + " (lenh " + l.value + " luc " + std::to_string(l.ms) + " ms)");
    };
    auto all_stopped_after = [&](long long since, const std::string &why) {
        for (const auto &l : ms) if (l.ms >= since && l.ms < o.stop_req_ms) ACC_CHECK(l.num <= 1e-3, why + " (lenh " + l.value + " luc " + std::to_string(l.ms) + " ms)");
    };

    if (kind == Brake) {  // tof_brake, tof_invalid, tof_brake_slow
        const TraceLine *z = first_zero_after(p->ms);
        ACC_CHECK(z != nullptr, "ToF truoc " + std::string(name == "tof_invalid" ? "= -1" : "= 100 mm < 250") + " khi dang tien: phai co lenh toc do 0");
        if (z) {
            const long long lat = z->ms - kT1;
            ACC_CHECK(lat >= -10, "xe dung luc " + std::to_string(z->ms) + " ms, truoc moc ToF doi " + std::to_string(kT1) + " ms (nguyen nhan khac?)");
            ACC_CHECK(lat <= bound, "phanh ToF mat " + std::to_string(lat) + " ms > " + std::to_string(bound) + " ms (= chu ky cam bien + chu ky dieu khien + 100 ms, hz=" + std::to_string(hz) + ")");
            all_stopped_after(z->ms, "sau khi phanh ToF khong duoc co lenh tien khi ToF truoc con gan/khong hop le");
        }
    } else if (kind == Reverse) {
        no_zero_between(p->ms, o.stop_req_ms, "dang lui thi ToF truoc gan khong duoc chan");
        bool after = false;
        for (const auto &l : ms) if (l.ms >= kT1 && acc::Near(l.num, -8.0)) after = true;
        ACC_CHECK(after, "sau khi ToF truoc thap van phai con lenh lui -8 (giu ket noi)");
    } else if (kind == Rear) {
        no_zero_between(p->ms, o.stop_req_ms, "chi ToF truoc moi duoc phanh tien; ToF sau/trai/phai gan thi khong chan");
        bool after = false;
        for (const auto &l : ms) if (l.ms >= kT1 && acc::Near(l.num, 8.0)) after = true;
        ACC_CHECK(after, "sau khi ToF sau/trai/phai thap van phai con lenh tien 8");
    } else if (kind == Order) {
        // tof_names = [rear, front, left, right]: cột 0 là rear, cột 1 là front.
        no_zero_between(p->ms, kT2 - 20, "cot 0 = rear: ToF 100 mm o cot dau khong duoc phanh khi front van xa");
        const TraceLine *z = first_zero_after(kT2 - 10);
        ACC_CHECK(z != nullptr, "cot 1 = front = 100 mm: phai phanh");
        if (z) ACC_CHECK(z->ms - kT2 <= bound, "phanh ToF (thu tu tof_names) mat " + std::to_string(z->ms - kT2) + " ms > " + std::to_string(bound));
    } else if (kind == Boundary) {
        no_zero_between(p->ms, kT2 - 20, "ToF truoc = 250 mm (= nguong, 'nho hon' nguong moi phanh) khong duoc phanh");
        const TraceLine *z = first_zero_after(kT2 - 10);
        ACC_CHECK(z != nullptr, "ToF truoc = 249 mm < 250: phai phanh");
        if (z) ACC_CHECK(z->ms - kT2 <= bound, "phanh ToF 249 mm mat " + std::to_string(z->ms - kT2) + " ms > " + std::to_string(bound));
    }
    return acc::Finish(("acc_hwtrace " + name).c_str());
}

// ---------------------------------------------------------------------------------------------------------------------
// Ca 3: camera ngắt rồi cấp lại (UCR_FAKE_CAMERA_GAPS). Trong lúc ngắt: dừng, không gửi lệnh chuyển động.
// Sau khi có ảnh lại: tài liệu (docs/ucr_car.md) nói "cho phép bộ điều khiển tiếp tục phát lệnh qua lớp an toàn" – không có
// yêu cầu giữ dừng, nên test chỉ GHI NHẬN hành vi này, không đòi hỏi.
int CameraGapScenario(const std::string &name) {
    Ctx x = Prepare();
    const int timeout = name == "camera_gap_200" ? 200 : 500;
    const int trim = 5;
    x.c.image_timeout_ms = timeout;
    x.c.servo_trim_deg = trim;
    const std::vector<std::pair<long long, long long>> gaps = {{2500, 3800}, {5300, 6100}};
    setenv("UCR_FAKE_CAMERA_GAPS", "2500:3800,5300:6100", 1);
    auto ctl = std::make_unique<FnController>([](long long, long long) { return common::ControlCommand{8.0f, 5}; });
    const auto o = RunApp(x, "auto", std::move(ctl), 7000);
    const Trace t = CommonChecks(x, o);
    if (o.begin_ms > 2000) return Inconclusive("khoi dong qua cham, chua chay truoc khi ngat camera");
    double v = 0;
    if (!StateAt(t, "motor_speed", gaps[0].first - 50, v) || !acc::Near(v, 8.0)) return Inconclusive("xe khong chay truoc khi camera ngat");

    // Bằng chứng camera giả thực sự bị ngắt: tuổi ảnh trong log.csv đạt xấp xỉ độ dài khoảng ngắt.
    const std::string run = acc::FindRunDir(x.logdir, "_auto");
    const acc::Csv csv = acc::ReadCsv(run + "/log.csv");
    double max_age = 0;
    for (size_t i = 0; i < csv.rows.size(); ++i) max_age = std::max(max_age, csv.Num(i, "image_age_ms"));
    ACC_CHECK(max_age >= 0.8 * (gaps[0].second - gaps[0].first), "UCR_FAKE_CAMERA_GAPS khong co tac dung: tuoi anh lon nhat trong log chi " + std::to_string(max_age) + " ms");

    for (size_t g = 0; g < gaps.size(); ++g) {
        const long long b = gaps[g].first, e = gaps[g].second;
        const long long deadline = b + timeout + 1000 / x.c.control_hz + 100;
        double before = 0;
        const bool moving_before = StateAt(t, "motor_speed", b - 50, before) && before > 0.5;
        if (moving_before) {
            double sp = 1e9, sv = 1e9;
            ACC_CHECK(StateAt(t, "motor_speed", deadline, sp) && acc::Near(sp, 0.0),
                      "khoang ngat " + std::to_string(g + 1) + ": den " + std::to_string(deadline) + " ms (ngat + image_timeout " + std::to_string(timeout) + " + 1 chu ky + 100) lenh toc do phai la 0, thay " + std::to_string(sp));
            ACC_CHECK(StateAt(t, "servo_angle", deadline, sv) && acc::Near(sv, trim),
                      "khoang ngat " + std::to_string(g + 1) + ": servo phai ve trim=" + std::to_string(trim) + " sau mat anh, thay " + std::to_string(sv));
        }
        for (const auto &l : t) {
            if (l.ms <= deadline || l.ms >= e) continue;
            if (l.kind == "motor_speed") ACC_CHECK(acc::Near(l.num, 0.0), "khoang ngat " + std::to_string(g + 1) + ": khong duoc gui lenh chuyen dong khi dang ngat, thay motor_speed " + l.value + " luc " + std::to_string(l.ms) + " ms");
            if (l.kind == "servo_angle") ACC_CHECK(acc::Near(l.num, trim), "khoang ngat " + std::to_string(g + 1) + ": servo phai o trim khi dang ngat, thay " + l.value + " luc " + std::to_string(l.ms) + " ms");
        }
        double after = 1e9;
        if (StateAt(t, "motor_speed", e + 600, after))
            std::printf("[GHI NHAN] sau khoang ngat %zu (ket thuc %lld ms), lenh toc do luc +600 ms = %g (tai lieu: anh tro lai cho phep bo dieu khien tiep tuc phat lenh)\n", g + 1, e, after);
    }
    CheckTail(o.after_wait, trim, "sau Wait");
    return acc::Finish(("acc_hwtrace " + name).c_str());
}

}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "Dung: acc_hwtrace <kich_ban>\n"); return 2; }
    const std::string s = argv[1];
    if (s == "servo_trim" || s == "servo_invert" || s == "servo_edge_pos" || s == "servo_edge_neg") return ServoScenario(s);
    if (s == "tail_normal" || s == "tail_destroy" || s == "tail_ctl_throw" || s == "tail_actuator_throw" || s == "tail_sensor_throw") return TailScenario(s);
    if (s == "keepalive_100" || s == "keepalive_200" || s == "keepalive_steer") return KeepaliveScenario(s);
    if (s == "speed_change") return SpeedChangeScenario();
    if (s == "init_once") return InitOnceScenario();
    if (s.compare(0, 4, "tof_") == 0) return TofScenario(s);
    if (s == "camera_gap" || s == "camera_gap_200") return CameraGapScenario(s);
    std::fprintf(stderr, "kich ban la: %s\n", s.c_str());
    return 2;
}
