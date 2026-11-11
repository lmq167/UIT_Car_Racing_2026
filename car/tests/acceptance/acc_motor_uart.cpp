// Test chấp nhận – không gửi lệnh cấu hình, chuỗi khởi tạo motor đúng 1 lần, lệnh tốc độ
// không chờ phản hồi).
//
// Phần cứng giả: cổng UART giả là một pty (cặp cổng ảo của Linux). Lớp Motor (giao diện công khai include/hardware/Motor.h)
// mở đầu "slave" như /dev/ttyTHS1; test giữ đầu "master", đọc lại MỌI byte Motor gửi ra và đóng vai driver (có thể im lặng,
// trả lời trễ 400 ms, hoặc tràn ngập dữ liệu rác liên tục).
//
// Yêu cầu gốc (không lấy từ mã nguồn):
//  - Quy tắc cứng của dự án: chuỗi khởi tạo motor BTC (Motor::Init -> SendInitCommand, có SaveAndResetDriver) chỉ chạy
//    ĐÚNG MỘT lần lúc khởi động chương trình chính; không gọi lại ở mở lại UART, luồng odometry, xử lý lỗi;
//    không AutoTunePid.
//  - Yêu cầu khởi tạo motor: "chống gọi lặp"; hàm chỉ mở cổng: hàm chỉ-mở-cổng (OpenPortOnly) không gửi lệnh `$...`.
//  - Yêu cầu lệnh tốc độ: "gửi lệnh tốc độ ... không chờ phản hồi (SendCommand(cmd,false) hoặc tương đương) để luồng Actuator
//    không bị chậm" (code mẫu BTC: ~110 ms mỗi lệnh).
//  - btc_code_notes.md mục 3: dạng lệnh `N1 v<rad/s>` (rad/s = cm/s / 3,25), `N1 O G1` (đọc vị trí, phản hồi `N1 p<rad>`,
//    đổi sang cm bằng rad x 3,25), chuỗi init: `$008=0`, `$002=330`, `O M3`, `$101=1`, rồi `O U C r`; `N1 O T` là tự dò PID (cấm).
#include "acc_common.h"
#include "hardware/Motor.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <termios.h>

namespace {

enum class Responder { Silent, Normal, Delayed, Flood };

struct Wire {
    std::string bytes;
    int status = -1;       // trạng thái waitpid
    bool timed_out = false;
    bool pty_failed = false;
    std::vector<std::string> Lines() const {
        std::vector<std::string> out;
        for (auto &l : acc::Split(bytes, '\n')) if (!l.empty()) out.push_back(l);
        return out;
    }
    bool Exited0() const { return WIFEXITED(status) && WEXITSTATUS(status) == 0; }
    std::string Describe() const {
        if (timed_out) return "het thoi gian (bi giet)";
        if (WIFSIGNALED(status)) return "bi tin hieu " + std::to_string(WTERMSIG(status));
        if (WIFEXITED(status)) return "thoat ma " + std::to_string(WEXITSTATUS(status));
        return "khong ro";
    }
};

// Chạy `child_body(duong_dan_slave)` trong tiến trình con (fork), cho đóng vai driver ở đầu master.
Wire RunChild(const std::function<int(const char *)> &child_body, Responder mode, int deadline_ms = 25000) {
    Wire w;
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) { w.pty_failed = true; return w; }
    const char *sn = ptsname(master);
    if (!sn) { w.pty_failed = true; return w; }
    const std::string slave = sn;
    // Cha tự giữ một đầu slave ở chế độ raw: tránh EIO khi con đóng cổng và tránh echo ký tự.
    int keep = open(slave.c_str(), O_RDWR | O_NOCTTY);
    if (keep >= 0) { termios t; if (tcgetattr(keep, &t) == 0) { cfmakeraw(&t); tcsetattr(keep, TCSANOW, &t); } }

    std::fflush(stdout); std::fflush(stderr);
    pid_t pid = fork();
    if (pid == 0) {
        close(master);
        if (keep >= 0) close(keep);
        int rc = child_body(slave.c_str());
        std::fflush(stdout); std::fflush(stderr);
        _exit(rc);
    }
    fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);

    struct Pending { std::chrono::steady_clock::time_point due; std::string data; };
    std::vector<Pending> queue;
    std::string partial;
    const auto start = std::chrono::steady_clock::now();
    bool child_done = false;
    auto drain_until = start;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (!child_done) {
            int st = 0;
            pid_t r = waitpid(pid, &st, WNOHANG);
            if (r == pid) { child_done = true; w.status = st; drain_until = now + std::chrono::milliseconds(150); }
            else if (now - start > std::chrono::milliseconds(deadline_ms)) {
                kill(pid, SIGKILL); waitpid(pid, &st, 0); w.status = st; w.timed_out = true; child_done = true;
                drain_until = now;
            }
        } else if (now >= drain_until) break;

        pollfd p{master, POLLIN, 0};
        if (poll(&p, 1, 1) > 0 && (p.revents & POLLIN)) {
            char buf[4096];
            ssize_t n = read(master, buf, sizeof buf);
            if (n > 0) {
                w.bytes.append(buf, buf + n);
                partial.append(buf, buf + n);
                size_t nl;
                while ((nl = partial.find('\n')) != std::string::npos) {
                    std::string line = partial.substr(0, nl);
                    partial.erase(0, nl + 1);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    std::string reply;
                    if (line.find("O G1") != std::string::npos) reply = "N1 p0.5\r\n";
                    else reply = "OK\r\n";
                    if (mode == Responder::Normal) {
                        if (write(master, reply.data(), reply.size()) < 0) { /* đầy bộ đệm: bỏ */ }
                    } else if (mode == Responder::Delayed) {
                        queue.push_back({now + std::chrono::milliseconds(400), reply});
                    }
                }
            }
        }
        if (mode == Responder::Delayed) {
            for (size_t i = 0; i < queue.size();) {
                if (std::chrono::steady_clock::now() >= queue[i].due) {
                    if (write(master, queue[i].data.data(), queue[i].data.size()) < 0) {}
                    queue.erase(queue.begin() + static_cast<long>(i));
                } else ++i;
            }
        }
        if (mode == Responder::Flood && !child_done) {
            static const char junk[] = "N1 p0.1\r\nxxxxxxxxxxxxxxxxxxxxxxxx\r\n";
            if (write(master, junk, sizeof junk - 1) < 0) {}
        }
    }
    if (keep >= 0) close(keep);
    close(master);
    return w;
}

size_t Count(const std::string &hay, const std::string &needle) {
    size_t n = 0, pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
    return n;
}

// Phân tích dòng "N1 v<số>" => true và trả số rad/s.
bool ParseSpeedLine(const std::string &line, double &v) {
    if (line.compare(0, 4, "N1 v") != 0) return false;
    const char *s = line.c_str() + 4;
    char *end = nullptr;
    v = std::strtod(s, &end);
    return end != s && *end == '\0';
}

// ---------------------------------------------------------------------------------------------------------------
// Ca 7: lệnh tốc độ không chờ phản hồi + đúng khuôn dạng lệnh.
void SpeedCommandsDoNotWait(Responder mode, const char *label) {
    auto body = [](const char *slave) -> int {
        Motor m(slave);
        if (!m.OpenPortOnly()) return 3;
        std::vector<double> samples;
        for (int i = 0; i < 10; ++i) {
            const float v = -20.f + 5.f * static_cast<float>(i);
            const auto t0 = std::chrono::steady_clock::now();
            m.SetSpeedCms(v);
            const double dt = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            samples.push_back(dt);
        }
        m.SetSpeedCms(6.5f);  // kết thúc bằng 6.5 cm/s => 2.0 rad/s
        std::sort(samples.begin(), samples.end());
        const double median = (samples[4] + samples[5]) / 2.0;
        const double p90 = samples[8];  // Hạng gần nhất: ceil(0.9 * 10) - 1.
        std::fprintf(stderr, "    [con] trung vi = %.1f ms, phan vi 90 = %.1f ms, max = %.1f ms\n",
                     median, p90, samples.back());
        return median < 50.0 && p90 < 50.0 && samples.back() < 100.0 ? 0 : 10;
    };
    Wire w = RunChild(body, mode);
    ACC_CHECK(!w.pty_failed, "tao duoc pty");
    if (w.pty_failed) return;
    ACC_CHECK(w.Exited0(), std::string(label) + ": SetSpeedCms phai co trung vi va phan vi 90 < 50 ms, max < 100 ms (khong cho phan hoi UART; ma BTC ~110 ms); con " + w.Describe());
    const auto lines = w.Lines();
    double last = 1e9;
    bool all_ok = true;
    int nspeed = 0;
    for (const auto &l : lines) {
        double v;
        if (!ParseSpeedLine(l, v)) { all_ok = false; ACC_CHECK(false, std::string(label) + ": dong khong phai 'N1 v<so>': '" + l + "'"); continue; }
        ++nspeed; last = v;
        // mọi giá trị phải là cm/s/3.25 của một trong các lệnh đã gọi (bội của 5/3.25, hoặc 2.0)
        bool known = std::fabs(v - 2.0) < 0.02;
        for (int i = 0; i < 10; ++i) if (std::fabs(v - (-20.0 + 5.0 * i) / 3.25) < 0.02) known = true;
        ACC_CHECK(known, std::string(label) + ": toc do rad/s la khong khop cm/s/3.25: " + l);
    }
    (void)all_ok;
    ACC_CHECK(nspeed >= 1, std::string(label) + ": phai co it nhat mot lenh toc do tren duong truyen");
    ACC_CHECK(std::fabs(last - 2.0) < 0.02, std::string(label) + ": lenh toc do cuoi (6.5 cm/s) phai la v=2.0 rad/s, nhan " + std::to_string(last));
}

// ---------------------------------------------------------------------------------------------------------------
// Ca 5 (mức Motor): đường "chỉ mở cổng" chỉ được gửi tốc độ / đọc vị trí, không lệnh cấu hình.
void OpenPortOnlySendsNoConfig() {
    auto body = [](const char *slave) -> int {
        Motor m(slave);
        if (!m.OpenPortOnly()) return 3;
        m.SetSpeedCms(0.f);
        float p = -1;
        bool ok = false;
        for (int i = 0; i < 5 && !ok; ++i) ok = m.GetPositionCm(&p);
        std::fprintf(stderr, "    [con] GetPositionCm ok=%d p=%.4f\n", ok ? 1 : 0, p);
        if (!ok) return 11;
        if (std::fabs(p - 1.625f) > 0.02f) return 12;  // 0.5 rad x 3.25 cm/rad
        m.SetSpeedCms(0.f);
        return 0;
    };
    Wire w = RunChild(body, Responder::Normal);
    ACC_CHECK(!w.pty_failed, "tao duoc pty");
    if (w.pty_failed) return;
    ACC_CHECK(w.Exited0(), "OpenPortOnly + SetSpeedCms(0) + GetPositionCm: phai doc duoc vi tri 0.5 rad = 1.625 cm; con " + w.Describe());
    ACC_CHECK(w.bytes.find('$') == std::string::npos, "duong chi-mo-cong KHONG duoc gui lenh '$...' (cau hinh/ROM)");
    int speed0 = 0;
    for (const auto &l : w.Lines()) {
        double v;
        if (ParseSpeedLine(l, v)) { if (std::fabs(v) < 1e-6) ++speed0; continue; }
        ACC_CHECK(l == "N1 O G1", "chi duoc gui toc do va 'N1 O G1' (doc vi tri); gap lenh la: '" + l + "'");
    }
    ACC_CHECK(speed0 >= 1, "OpenPortOnly + SetSpeedCms(0) phai gui 'N1 v0' (toc do 0) len duong truyen");
}

// ---------------------------------------------------------------------------------------------------------------
// Ca 6: chuỗi khởi tạo đúng một lần mỗi lần khởi động chương trình.
void InitCountChecks(const Wire &w, const std::string &label) {
    ACC_CHECK(!w.pty_failed, "tao duoc pty");
    if (w.pty_failed) return;
    ACC_CHECK(!w.timed_out, label + ": khong duoc treo (" + w.Describe() + ")");
    ACC_CHECK(Count(w.bytes, "$101=1") == 1, label + ": '$101=1' (luu ROM + reset) phai xuat hien DUNG 1 lan, thay " + std::to_string(Count(w.bytes, "$101=1")));
    ACC_CHECK(Count(w.bytes, "$008=0") <= 1, label + ": '$008=0' toi da 1 lan");
    ACC_CHECK(Count(w.bytes, "$002=330") <= 1, label + ": '$002=330' toi da 1 lan");
    bool has_tune = false;
    for (const auto &l : w.Lines()) if (l == "N1 O T") has_tune = true;
    ACC_CHECK(!has_tune, label + ": AutoTunePid ('N1 O T') bi CAM");
}

void InitRunsExactlyOnce() {
    // A: gọi Init() hai lần trên cùng đối tượng.
    {
        auto body = [](const char *slave) -> int {
            Motor m(slave);
            const bool r1 = m.Init();
            const bool r2 = m.Init();
            std::fprintf(stderr, "    [con] Init lan 1 = %d, lan 2 = %d\n", r1 ? 1 : 0, r2 ? 1 : 0);
            return 0;
        };
        Wire w = RunChild(body, Responder::Normal);
        InitCountChecks(w, "Init() hai lan");
        // Chuỗi nguyên bản của BTC (btc_code_notes mục 3): $008 -> $002 -> O M3 -> $101=1 -> O U C r.
        const auto p008 = w.bytes.find("$008=0"), p002 = w.bytes.find("$002=330"), pm3 = w.bytes.find("O M3"),
                   p101 = w.bytes.find("$101=1"), pucr = w.bytes.find("O U C r");
        ACC_CHECK(p008 != std::string::npos && p002 != std::string::npos && pm3 != std::string::npos &&
                  p101 != std::string::npos && pucr != std::string::npos,
                  "lan Init dau tien phai gui du chuoi BTC: $008=0, $002=330, O M3, $101=1, O U C r (giu nguyen)");
        if (p008 != std::string::npos && p002 != std::string::npos && pm3 != std::string::npos &&
            p101 != std::string::npos && pucr != std::string::npos) {
            ACC_CHECK(p008 < p002 && p002 < pm3 && pm3 < p101 && p101 < pucr, "thu tu chuoi khoi tao BTC bi thay doi");
        }
    }
    // B: Init, huỷ đối tượng (đóng cổng), tạo đối tượng mới mở lại cổng, rồi Init lần nữa.
    {
        auto body = [](const char *slave) -> int {
            { Motor a(slave); a.Init(); }
            Motor b(slave);
            if (!b.OpenPortOnly()) return 3;
            b.SetSpeedCms(0.f);
            b.Init();
            return 0;
        };
        InitCountChecks(RunChild(body, Responder::Normal), "Init, dong cong, mo lai + Init");
    }
    // C: Init một lần, sau đó odometry (GetPositionCm) lặp lại + lệnh tốc độ + mở lại cổng bằng đối tượng khác.
    {
        auto body = [](const char *slave) -> int {
            Motor m(slave);
            m.Init();
            float p;
            for (int i = 0; i < 6; ++i) { m.GetPositionCm(&p); m.SetSpeedCms(1.0f * static_cast<float>(i)); }
            Motor c(slave);
            if (c.OpenPortOnly()) { c.SetSpeedCms(0.f); c.GetPositionCm(&p); }
            return 0;
        };
        InitCountChecks(RunChild(body, Responder::Normal), "Init + odometry + mo lai cong");
    }
    // D: Init khi driver im lặng (không phản hồi): vẫn chỉ một chuỗi; không thử lại cấu hình khi lỗi.
    {
        auto body = [](const char *slave) -> int {
            Motor m(slave);
            m.Init();
            m.Init();
            return 0;
        };
        InitCountChecks(RunChild(body, Responder::Silent), "Init khi driver im lang");
    }
}

}  // namespace

int main() {
    // Nhãn acceptance không cho phép bỏ qua; thiếu pty phải báo HỎNG rõ nguyên nhân.
    {
        int m = posix_openpt(O_RDWR | O_NOCTTY);
        if (m < 0) { std::fprintf(stderr, "HONG: moi truong khong tao duoc pty\n"); return 1; }
        close(m);
    }
    SpeedCommandsDoNotWait(Responder::Silent, "driver im lang");
    SpeedCommandsDoNotWait(Responder::Delayed, "driver tra loi tre 400 ms");
    SpeedCommandsDoNotWait(Responder::Flood, "driver tran ngap du lieu lien tuc");
    OpenPortOnlySendsNoConfig();
    InitRunsExactlyOnce();
    return acc::Finish("acc_motor_uart");
}
