// Test chấp nhận – thoát bình thường và một phần mất ảnh ở mức lớp App, chạy với UCR_FAKE_HW=ON.
//
// Yêu cầu gốc:
//  - Quy tắc an toàn khi chạy xe (a): mọi chương trình điều khiển xe phải dừng motor + trả servo về 0 khi thoát.
//  - Yêu cầu an toàn của chương trình xe: "mọi đường thoát (Ctrl+C, lỗi, std::terminate) -> motor 0, servo về trim"; "dừng sạch".
//  - docs/ucr_car.md: `manual` không có màn hình thì "xe giữ dừng"; `auto` hiện chỉ có StopController (luôn lệnh dừng);
//    cột log.csv: t,frame_id,image_age_ms,cmd_speed,cmd_steer,... (cmd_* là lệnh SAU lớp an toàn).
// Giao diện dùng: include/app/App.h (App, Init, Start, RequestStop, Wait, IsRunning, Failed), common/Config.h.
// Không quan sát được "lệnh cuối gửi phần cứng" (bản giả không ghi lại lệnh) => dùng log.csv làm bằng chứng gián tiếp;
// xem mục "Giao diện còn thiếu" trong báo cáo.
#include "acc_common.h"
#include "app/App.h"
#include "common/Config.h"
#include <future>
#include <memory>

namespace {

struct Env {
    std::string root, logdir, cfgpath;
    common::Config c;
};

Env MakeEnv() {
    Env e;
    e.root = acc::MakeTempDir("acc_app_");
    e.logdir = e.root + "/runs";
    mkdir(e.logdir.c_str(), 0755);
    e.cfgpath = e.root + "/config.yaml";
    acc::WriteFile(e.cfgpath, "%YAML:1.0\n---\nlog_dir: \"" + e.logdir + "\"\nenable_display: 0\nenable_oled: 0\nfake_source: \"\"\n");
    e.c.log_dir = e.logdir;
    e.c.enable_display = 0;
    e.c.enable_oled = 0;
    e.c.fake_source = "";
    return e;
}

template <class R>
bool WaitOrDie(std::future<R> &f, int seconds, const char *what) {
    if (f.wait_for(std::chrono::seconds(seconds)) != std::future_status::ready) {
        std::fprintf(stderr, "[HONG] %s: khong ket thuc sau %d s (treo)\n", what, seconds);
        std::fflush(stderr);
        _exit(1);  // treo => thất bại ngay, không đợi destructor
    }
    return true;
}

struct RunResult { bool init_ok = true; bool running_mid = false; bool running_after = true; bool failed = false; long stop_ms = -1; std::string error; };

RunResult RunApp(const Env &e, const std::string &mode, int run_ms) {
    RunResult r;
    try {
        app::App app(e.c, mode, e.cfgpath);
        app.Init();
        app.Start();
        acc::SleepMs(run_ms);
        r.running_mid = app.IsRunning();
        const auto t0 = std::chrono::steady_clock::now();
        app.RequestStop();
        app.Wait();
        r.stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        r.running_after = app.IsRunning();
        r.failed = app.Failed();
    } catch (const std::exception &ex) {
        r.init_ok = false; r.error = ex.what();
    } catch (...) { r.init_ok = false; r.error = "ngoai le khong ro"; }
    return r;
}

void CheckLogZeroCommands(const std::string &dir_suffix, const Env &e, const std::string &label, size_t min_rows) {
    const std::string run = acc::FindRunDir(e.logdir, dir_suffix);
    ACC_CHECK(!run.empty(), label + ": phai tao thu muc runs/..." + dir_suffix);
    if (run.empty()) return;
    ACC_CHECK(acc::Exists(run + "/log.csv"), label + ": phai co log.csv");
    ACC_CHECK(acc::Exists(run + "/config.yaml"), label + ": phai chep config.yaml dang dung vao thu muc luot chay");
    const acc::Csv csv = acc::ReadCsv(run + "/log.csv");
    for (const char *col : {"t", "frame_id", "image_age_ms", "cmd_speed", "cmd_steer", "meas_speed", "odom_cm", "yaw_rel",
                            "pitch", "roll", "tof0", "tof1", "tof2", "tof3", "state", "fps"})
        ACC_CHECK(csv.Col(col) >= 0, label + ": log.csv thieu cot " + col + " (yeu cau cot log.csv)");
    ACC_CHECK(csv.rows.size() >= min_rows, label + ": log.csv phai co >= " + std::to_string(min_rows) + " dong, co " + std::to_string(csv.rows.size()));
    for (size_t i = 0; i < csv.rows.size(); ++i) {
        const double s = csv.Num(i, "cmd_speed"), a = csv.Num(i, "cmd_steer");
        if (!(s == 0.0 && a == 0.0)) {
            ACC_CHECK(false, label + ": dong " + std::to_string(i) + " cmd_speed=" + std::to_string(s) + " cmd_steer=" + std::to_string(a) + " (phai 0 vi chi co StopController / khong co man hinh)");
            break;
        }
    }
}

}  // namespace

int main() {
    // --- auto: chạy 1,5 s rồi yêu cầu dừng
    {
        Env e = MakeEnv();
        auto f = std::async(std::launch::async, [&] { return RunApp(e, "auto", 1500); });
        WaitOrDie(f, 20, "auto: RequestStop + Wait");
        RunResult r = f.get();
        ACC_CHECK(r.init_ok, "auto: Init/Start khong duoc nem loi tren ban gia lap: " + r.error);
        ACC_CHECK(r.running_mid, "auto: dang chay sau 1.5 s (chua tu dung)");
        ACC_CHECK(r.stop_ms >= 0 && r.stop_ms <= 3000, "auto: RequestStop -> Wait phai xong trong <= 3 s, mat " + std::to_string(r.stop_ms) + " ms");
        ACC_CHECK(!r.running_after, "auto: sau Wait, IsRunning() phai false");
        ACC_CHECK(!r.failed, "auto: dung binh thuong khong duoc danh dau Failed()");
        CheckLogZeroCommands("_auto", e, "auto", 5);
    }
    // --- manual không có màn hình: xe giữ dừng
    {
        Env e = MakeEnv();
        auto f = std::async(std::launch::async, [&] { return RunApp(e, "manual", 1000); });
        WaitOrDie(f, 20, "manual: RequestStop + Wait");
        RunResult r = f.get();
        ACC_CHECK(r.init_ok, "manual: Init/Start khong duoc nem loi: " + r.error);
        ACC_CHECK(r.stop_ms >= 0 && r.stop_ms <= 3000, "manual: dung trong <= 3 s");
        ACC_CHECK(!r.running_after, "manual: sau Wait, IsRunning() phai false");
        CheckLogZeroCommands("_manual", e, "manual (khong man hinh)", 3);
    }
    // --- huỷ đối tượng khi đang chạy, không gọi RequestStop/Wait: không được treo hay terminate
    {
        Env e = MakeEnv();
        auto f = std::async(std::launch::async, [&] {
            app::App app(e.c, "auto", e.cfgpath);
            app.Init();
            app.Start();
            acc::SleepMs(600);
            return true;  // ~App chạy ở đây
        });
        WaitOrDie(f, 20, "auto: huy App khi dang chay");
        ACC_CHECK(f.get(), "huy App khi dang chay xong binh thuong");
        CheckLogZeroCommands("_auto", e, "huy khi dang chay", 3);
    }
    return acc::Finish("acc_app_lifecycle");
}
