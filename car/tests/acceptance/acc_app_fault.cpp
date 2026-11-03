// Test chấp nhận – giới hạn tốc độ/góc đầu-cuối, mất ảnh đầu-cuối và ngoại lệ trong luồng tự lái, chạy lớp App thật ở
// UCR_FAKE_HW=ON với một bộ điều khiển (IController) do test tiêm vào bằng App::SetController (giao diện công khai).
//
// Nếu bộ điều khiển tiêm vào chưa từng được gọi (điều kiện tiền đề không xảy ra) thì test trả mã 1 khi thiếu tiền điều kiện
// dưới nhãn acceptance phải HỎNG, không bỏ qua. Nếu đã được gọi mà hành vi sai thì là HỎNG.
//
// Yêu cầu gốc:
//  - Quy tắc an toàn khi chạy xe: (a) dừng motor + servo về 0 khi lỗi và khi mất ảnh quá 0,5 s; (b) giới hạn tốc độ tối
//    đa trong config.
//  - Yêu cầu an toàn của chương trình xe: chặn tốc độ/góc theo config; mất ảnh > 0,5 s ở chế độ auto -> dừng; mọi đường thoát (lỗi) -> motor 0.
//  - docs/ucr_car.md: "Lỗi đọc cảm biến, ghi lệnh, ghi log hay ngoại lệ trong luồng yêu cầu dừng"; log.csv ghi lệnh SAU
//    lớp an toàn; image_timeout_ms mặc định 500 => mất ảnh trả tốc độ và góc tương đối về 0; nguồn ảnh giả (chuỗi ảnh) hết
//    thì không lặp lại.
// Dùng: acc_app_fault <throw|clamp_pos|clamp_neg|nan|pass|imgloss>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "acc_common.h"
#include "acc_seed_gate.h"
#include "app/Backend.h"
#include "app/IController.h"
#include "common/Config.h"
#include "common/RunLogger.h"
#include "common/SharedChannels.h"
#include "app/App.h"

namespace {

struct InjectedController : public app::IController {
    float speed = 0;
    int steer = 0;
    bool do_throw = false;
    std::shared_ptr<std::atomic<int>> calls = std::make_shared<std::atomic<int>>(0);  // sống lâu hơn App
    common::ControlCommand step(const common::Observation &) override {
        ++*calls;
        if (do_throw) throw std::runtime_error("loi gia lap trong luong tu lai");
        return {speed, steer};
    }
};

bool Near(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "Dung: acc_app_fault <kich_ban>\n"); return 2; }
    const std::string scenario = argv[1];

    const std::string root = acc::MakeTempDir("acc_fault_");
    if (root.empty() || setenv("UCR_FAKE_HW_TRACE", (root + "/trace.csv").c_str(), 1) != 0) {
        std::perror("khong tao duoc tep vet rieng cho luot kiem tra");
        return 2;
    }
    const std::string logdir = root + "/runs";
    mkdir(logdir.c_str(), 0755);
    const std::string cfgpath = root + "/config.yaml";
    acc::WriteFile(cfgpath, "%YAML:1.0\n---\nlog_dir: \"" + logdir + "\"\nenable_display: 0\nenable_oled: 0\n");

    common::Config c;
    c.log_dir = logdir;
    c.enable_display = 0;
    c.enable_oled = 0;
    c.max_speed_cms = 12.5f;
    c.max_steering_deg = 10;
    c.fake_source = "";
    c.tof_mapping_confirmed=1;

    auto *ctl = new InjectedController();
    const auto calls = ctl->calls;
    int run_ms = 1500;
    if (scenario == "throw") ctl->do_throw = true;
    else if (scenario == "clamp_pos") { ctl->speed = 1000.f; ctl->steer = 1000; }
    else if (scenario == "clamp_neg") { ctl->speed = -1000.f; ctl->steer = -1000; }
    else if (scenario == "nan") { ctl->speed = std::numeric_limits<float>::quiet_NaN(); ctl->steer = 7; }
    else if (scenario == "pass") { ctl->speed = 6.f; ctl->steer = 4; }
    else if (scenario == "imgloss") {
        // E4: nguồn 20 ảnh ở 10 Hz đủ tăng mềm 1 s rồi mới mất ảnh.
        const std::string imgdir = root + "/anh thu";
        mkdir(imgdir.c_str(), 0755);
        for (int i = 0; i < 20; ++i) {
            cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(40 + 30 * i, 100, 160));
            cv::circle(img, cv::Point(80 + 70 * i, 190), 40, cv::Scalar(255, 255, 255), -1);
            char name[64];
            std::snprintf(name, sizeof name, "/frame_%06d.jpg", i);
            if (!cv::imwrite(imgdir + name, img)) { std::fprintf(stderr, "khong ghi duoc anh\n"); return 2; }
        }
        c.fake_source = imgdir + "/frame_%06d.jpg";
        c.camera_hz = 10;
        ctl->speed = 10.f; ctl->steer = 5;
        run_ms = 4500;
    } else { std::fprintf(stderr, "kich ban la: %s\n", scenario.c_str()); return 2; }

    // Tạo lại dữ liệu cổng sau khi đã chọn đầy đủ cấu hình của kịch bản.
    ACC_CHECK(acc::WriteFile(cfgpath,"%YAML:1.0\n---\nlog_dir: \""+logdir+
        "\"\nenable_display: 0\nenable_oled: 0\ntof_mapping_confirmed: 1\nmax_speed_cms: "+
        std::to_string(c.max_speed_cms)+"\nmax_steering_deg: "+std::to_string(c.max_steering_deg)+
        "\nfake_source: \""+c.fake_source+"\"\n"),"Ghi config đúng kịch bản trước mẫu smoke");
    acc::SeedGate(c,cfgpath);

    bool stopped_by_itself = false, failed = false;
    long wait_ms = -1;
    {
        auto fut = std::async(std::launch::async, [&] {
            app::App app(c, "auto", cfgpath);
            app.Init();
            app.SetController(std::unique_ptr<app::IController>(ctl));  // tiêm bộ điều khiển thử (trước Start)
            app.Start();
            const auto t0 = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(run_ms)) {
                if (scenario == "throw" && !app.IsRunning()) { stopped_by_itself = true; break; }
                acc::SleepMs(20);
            }
            if (scenario == "throw") { stopped_by_itself = stopped_by_itself || !app.IsRunning(); failed = app.Failed(); }
            const auto t1 = std::chrono::steady_clock::now();
            app.RequestStop();
            app.Wait();
            wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
            ctl = nullptr;  // App giữ quyền sở hữu; ~App huỷ (tránh dùng sau khi huỷ)
        });
        if (fut.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            std::fprintf(stderr, "[HONG] %s: khong ket thuc sau 30 s (treo)\n", scenario.c_str());
            std::fflush(stderr);
            _exit(1);
        }
        try { fut.get(); } catch (const std::exception &ex) {
            ACC_CHECK(false, std::string("ngoai le thoat ra khoi App: ") + ex.what());
        }
    }

    // ctl đã bị App huỷ; số lần gọi nằm trong bộ đếm dùng chung `calls`. Với các kịch bản lệnh khác 0, nếu log không có dòng
    // nào khác 0 thì coi như bộ điều khiển không được gọi (CHƯA KIỂM ĐƯỢC).
    const std::string run = acc::FindRunDir(logdir, "_auto");
    ACC_CHECK(!run.empty(), "phai co thu muc runs/..._auto");
    const acc::Csv csv = acc::ReadCsv(run.empty() ? "" : run + "/log.csv");
    // Kịch bản "throw": lỗi xảy ra ở khung đầu tiên nên có thể chưa kịp ghi dòng nào; chỉ yêu cầu có dữ liệu ở các kịch bản khác.
    if (scenario != "throw") ACC_CHECK(csv.rows.size() >= 1, "log.csv phai co du lieu");
    ACC_CHECK(wait_ms >= 0 && wait_ms <= 3000, "RequestStop -> Wait phai xong trong <= 3 s, mat " + std::to_string(wait_ms) + " ms");

    auto any_row = [&](const std::function<bool(double, double, double)> &pred) {
        for (size_t i = 0; i < csv.rows.size(); ++i)
            if (pred(csv.Num(i, "cmd_speed"), csv.Num(i, "cmd_steer"), csv.Num(i, "image_age_ms"))) return true;
        return false;
    };
    auto all_rows = [&](const std::function<bool(double, double, double)> &pred) {
        for (size_t i = 0; i < csv.rows.size(); ++i)
            if (!pred(csv.Num(i, "cmd_speed"), csv.Num(i, "cmd_steer"), csv.Num(i, "image_age_ms"))) return false;
        return true;
    };

    if (scenario == "throw") {
        // Chưa từng được gọi: điều kiện tiền đề không xảy ra => CHƯA KIỂM ĐƯỢC. Đã được gọi (và đã ném lỗi) mà App không tự dừng
        // và không Failed() thì là HỎNG (lỗi bị nuốt).
        if (calls->load() == 0) {
            std::fprintf(stderr, "Bo dieu khien chua tung duoc goi: HONG: tien de khong dat\n");
            return 1;
        }
        ACC_CHECK(stopped_by_itself, "ngoai le trong luong tu lai phai lam App tu yeu cau dung (IsRunning()==false) trong <= 1.5 s");
        ACC_CHECK(failed, "ngoai le trong luong tu lai phai danh dau Failed()==true");
        ACC_CHECK(all_rows([](double s, double a, double) { return Near(s,0.0) && a == 0.0; }), "moi dong log sau/trong su co phai co cmd = 0");
    } else if (scenario == "clamp_pos") {
        ACC_CHECK(all_rows([](double s, double a, double) { return s >= -1e-3 && s <= 12.5 + 1e-3 && a >= 0.0 && a <= 10.0; }),
                  "lenh +1000/+1000 khong duoc vuot (12.5, 10) o bat ky dong log nao");
        const bool reached = any_row([](double s, double a, double) { return Near(s, 12.5) && Near(a, 10.0); });
        if (!reached && !any_row([](double s, double, double) { return !Near(s,0.0); })) { std::fprintf(stderr, "Bo dieu khien khong duoc goi: HONG: tien de khong dat\n"); return 1; }
        ACC_CHECK(reached, "lenh +1000/+1000 phai bi chan DUNG (12.5, 10) (khong phai 0 hay gia tri khac)");
    } else if (scenario == "clamp_neg") {
        ACC_CHECK(all_rows([](double s, double a, double) { return s <= 1e-3 && s >= -12.5 - 1e-3 && a <= 0.0 && a >= -10.0; }),
                  "lenh -1000/-1000 khong duoc vuot (-12.5, -10)");
        const bool reached = any_row([](double s, double a, double) { return Near(s, -12.5) && Near(a, -10.0); });
        if (!reached && !any_row([](double s, double, double) { return !Near(s,0.0); })) { std::fprintf(stderr, "Bo dieu khien khong duoc goi: HONG: tien de khong dat\n"); return 1; }
        ACC_CHECK(reached, "lenh lui -1000/-1000 phai bi chan DUNG (-12.5, -10); ToF truoc xa nen khong duoc chan lui");
    } else if (scenario == "nan") {
        ACC_CHECK(all_rows([](double s, double a, double) { return std::isfinite(s) && std::isfinite(a) && std::fabs(s) <= 12.5 + 1e-3 && std::fabs(a) <= 10.0; }),
                  "toc do NaN tu bo dieu khien khong duoc lot ra log/phan cung duoi dang khong huu han hoac vuot gioi han");
    } else if (scenario == "pass") {
        const bool reached = any_row([](double s, double a, double) { return Near(s, 6.0) && Near(a, 4.0); });
        if (!reached && !any_row([](double s, double, double) { return !Near(s,0.0); })) { std::fprintf(stderr, "Bo dieu khien khong duoc goi: HONG: tien de khong dat\n"); return 1; }
        ACC_CHECK(reached, "lenh (6, 4) trong gioi han phai di qua nguyen ven");
        ACC_CHECK(all_rows([](double s, double a, double) { return s >= -1e-3 && s <= 6.0 + 1e-3 && a >= 0.0 && a <= 4.0 + 1e-3; }), "khong dong log nao vuot lenh yeu cau (6, 4)");
    } else if (scenario == "imgloss") {
        const bool moved = any_row([](double s, double a, double age) { return Near(s, 10.0) && Near(a, 5.0) && age < 500.0; });
        if (!moved) { std::fprintf(stderr, "Bo dieu khien khong duoc goi hoac khong co dong khi con anh: HONG: tien de khong dat\n"); return 1; }
        const bool saw_loss = any_row([](double, double, double age) { return age > 600.0; });
        ACC_CHECK(saw_loss, "trong cua so 4.5 s, log phai co dong voi image_age_ms > 600 sau khi nguon anh het");
        ACC_CHECK(all_rows([](double s, double a, double age) { return !(age > 600.0) || (Near(s,0.0) && a == 0.0); }),
                  "moi dong co image_age_ms > 600 (mat anh qua 0.5 s + 1 chu ky + du dia) phai co cmd_speed=0 va cmd_steer=0");
    }
    return acc::Finish(("acc_app_fault " + scenario).c_str());
}
