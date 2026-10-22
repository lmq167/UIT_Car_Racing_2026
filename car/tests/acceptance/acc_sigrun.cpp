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
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

int main(int argc, char **argv) {
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
    if (pid == 0) {
        int fd = open(outfile, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) { dup2(fd, 1); dup2(fd, 2); close(fd); }
        signal(SIGINT, SIG_DFL);  // nền không có job control có thể bỏ qua SIGINT: đặt lại mặc định cho công bằng
        execvp(cmd[0], cmd.data());
        _exit(127);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    int st = 0;
    const bool alive = waitpid(pid, &st, WNOHANG) == 0;
    const auto t0 = std::chrono::steady_clock::now();
    bool done = !alive;
    if (alive) kill(pid, sig);
    while (alive && !done) {
        if (waitpid(pid, &st, WNOHANG) == pid) { done = true; break; }
        if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(deadline_ms)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    std::printf("alive=%d\n", alive ? 1 : 0);
    if (!done) {
        kill(pid, SIGKILL); waitpid(pid, &st, 0);
        std::printf("result=hung\n");
    } else if (WIFEXITED(st)) std::printf("result=exited:%d\n", WEXITSTATUS(st));
    else if (WIFSIGNALED(st)) std::printf("result=signaled:%d\n", WTERMSIG(st));
    else std::printf("result=unknown\n");
    std::printf("elapsed_ms=%ld\n", elapsed);
    return 0;
}
