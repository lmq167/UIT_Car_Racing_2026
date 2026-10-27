// Công cụ phụ cho test chấp nhận: chạy một chương trình, chờ, gửi tín hiệu, đo thời gian dừng và phân biệt
// "tự thoát với mã N" (exited) với "bị tín hiệu giết" (signaled) – điều mà mã thoát 130/143 của bash không phân biệt được.
// Dùng: acc_sigrun <tín-hiệu: INT|TERM|KILL> <chờ-trước-khi-gửi-ms> <hạn-chờ-sau-khi-gửi-ms> <file-ghi-dầu-ra> -- lệnh [tham số...]
// In ra stdout: alive=<0|1> ; result=exited:<mã> | signaled:<số> | hung ; elapsed_ms=<ms>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <cmath>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "acc_common.h"
#include "acc_signal_timing.h"

bool LogReady() {
    const char *dir = std::getenv("ACC_READY_LOG_DIR");
    if (!dir) return true;
    const char *mode = std::getenv("ACC_READY_MODE");
    const auto run = acc::FindRunDir(dir, std::string("_") + (mode ? mode : "auto"));
    const auto csv = acc::ReadCsv(run + "/log.csv");
    if (csv.rows.size() < 5) return false;
    if (!std::getenv("ACC_READY_STALE")) return true;
    for (size_t i = 0; i < csv.rows.size(); ++i)
        if (csv.Num(i, "image_age_ms") > 600) return true;
    return false;
}

// Phiếu làm chắc test §1/§3: chờ bằng chứng phần cứng, không ngủ đoán lúc sẵn sàng.
// ACC_READY_FILE + ACC_READY_KIND=moving|init; ACC_SIGNAL_TRACE bật phép đo dừng.
bool Ready(const char *path, const std::string &kind) {
    std::ifstream in(path);
    std::string line;
    size_t moving = 0, servo = 0, init = 0;
    while (std::getline(in, line)) {
        if (in.eof()) break;  // Bỏ dòng đang ghi dở.
        std::istringstream fields(line);
        std::string ms, type, value;
        if (!std::getline(fields, ms, ',') || !std::getline(fields, type, ',') ||
            !std::getline(fields, value)) continue;
        char *end = nullptr;
        const double v = std::strtod(value.c_str(), &end);
        if (end == value.c_str() || *end || !std::isfinite(v)) continue;
        if (type == "motor_speed" && std::fabs(v) > 1e-3) ++moving;
        if (type == "servo_angle") ++servo;
        if (type == "motor_init") ++init;
    }
    return kind == "moving" ? moving >= 5 : kind == "init" && init == 1 && servo >= 5;
}

int main(int argc, char **argv) {
    if ((argc == 2 || argc == 3) && std::strcmp(argv[1], "--timing-self-test") == 0)
        return SignalTimingSelfTest(argc == 3 ? argv[2] : nullptr);
    if (argc < 7 || std::strcmp(argv[5], "--") != 0) {
        std::fprintf(stderr, "Dung: acc_sigrun <INT|TERM|KILL> <cho_ms> <han_ms> <file_dau_ra> -- lenh [tham so...]\n");
        return 2;
    }
    const std::string name = argv[1];
    const int sig = name == "INT" ? SIGINT : name == "TERM" ? SIGTERM : name == "KILL" ? SIGKILL : 0;
    if (!sig) { std::fprintf(stderr, "tin hieu khong hop le\n"); return 2; }
    const int wait_ms = std::atoi(argv[2]);
    const int deadline_ms = std::atoi(argv[3]);
    const char *outfile = argv[4];
    std::vector<char *> cmd(argv + 6, argv + argc);
    cmd.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) { std::perror("fork"); return 1; }
    if (pid == 0) {
        int fd = open(outfile, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) { std::perror("file dau ra"); _exit(126); }
        dup2(fd, 1); dup2(fd, 2); close(fd);
        signal(SIGINT, SIG_DFL);  // nền không có job control có thể bỏ qua SIGINT: đặt lại mặc định cho công bằng
        execvp(cmd[0], cmd.data());
        _exit(127);
    }
    int st = 0;
    const char *ready_file = std::getenv("ACC_READY_FILE");
    const char *kind_env = std::getenv("ACC_READY_KIND");
    const std::string kind = kind_env ? kind_env : "moving";
    bool ready = false, alive = true;
    const auto begin = std::chrono::steady_clock::now();
    while (true) {
        if (waitpid(pid, &st, WNOHANG) == pid) { alive = false; break; }
        if (ready_file) {
            if (Ready(ready_file, kind) && LogReady()) { ready = true; break; }
        } else if (std::chrono::steady_clock::now() - begin >= std::chrono::milliseconds(wait_ms)) {
            ready = true; break;
        }
        if (std::chrono::steady_clock::now() - begin >= std::chrono::seconds(10)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    SignalTiming timing;
    const char *trace = std::getenv("ACC_SIGNAL_TRACE");
    if (trace && !*trace) trace = nullptr;
    const char *budget_env = std::getenv("ACC_STOP_BUDGET_MS");
    const double budget = budget_env ? std::atof(budget_env) : 0;
    const char *period_env = std::getenv("ACC_CONTROL_PERIOD_MS");
    const double period = period_env ? std::atof(period_env) : 0;
    if (trace) { timing.path = trace; timing.Scan(false); }
    bool done = !alive;
    if (alive) {
        pid_t target = pid;
        const char *target_file = std::getenv("ACC_SIGNAL_TARGET_FILE");
        if (target_file && *target_file && ready) {
            std::ifstream in(target_file);
            long number = 0;
            if (!(in >> number) || number <= 1) ready = false;
            else target = static_cast<pid_t>(number);
        }
        // Thiếu tiền đề vẫn dọn con, nhưng không gửi tín hiệu đang cần kiểm.
        timing.signal = timing.Now();
        if (kill(ready ? target : pid, ready ? sig : SIGKILL) != 0) ready = false;
    }
    const auto t0 = std::chrono::steady_clock::now();
    while (alive && !done) {
        if (trace) timing.Scan(true);
        if (waitpid(pid, &st, WNOHANG) == pid) { done = true; break; }
        if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(deadline_ms)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(trace ? 1 : 10));
    }
    if (trace) timing.Scan(true);
    const long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    std::printf("alive=%d\n", alive ? 1 : 0);
    std::printf("ready=%d\n", ready ? 1 : 0);
    if (!done) {
        kill(pid, SIGKILL); waitpid(pid, &st, 0);
        std::printf("result=hung\n");
    } else if (WIFEXITED(st)) std::printf("result=exited:%d\n", WEXITSTATUS(st));
    else if (WIFSIGNALED(st)) std::printf("result=signaled:%d\n", WTERMSIG(st));
    else std::printf("result=unknown\n");
    std::printf("elapsed_ms=%ld\n", elapsed);
    if (trace) {
        timing.Print(budget, period, sig == SIGKILL);
        std::printf("timing_ok=%d\n", timing.Passed(budget, period, sig == SIGKILL) ? 1 : 0);
    }
    return ready ? 0 : 1;
}
