#pragma once
// Phần chạy của TUI (P5b): chỉ bật khi stdout là terminal. std::cout/std::cerr được gom thành dòng sự kiện; một luồng
// riêng <= 10 Hz vẽ khung bằng poll + write từng mẩu 256 byte (terminal nghẽn thì bỏ khung; Ctrl+S vẫn có thể giữ luồng này, không giữ vòng điều khiển). Vòng điều khiển chỉ gọi update()
// (chép vài số dưới khóa). Không đụng tín hiệu, phím hay đường dừng; warning() vẫn ghi thẳng stderr.
#include "tui.h"
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <streambuf>
#include <string>
#include <thread>

namespace auto_drive {
class TuiRuntime {
public:
    // Gom từng dòng của một luồng xuất thành sự kiện (an toàn đa luồng).
    class LineBuf : public std::streambuf {
    public:
        LineBuf(TuiRuntime& owner, bool warn) : owner_(owner), warn_(warn) {}
    protected:
        int overflow(int c) override {
            if (c == traits_type::eof()) return 0;
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            put(char(c));
            return c;
        }
        std::streamsize xsputn(const char* s, std::streamsize n) override {
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            for (std::streamsize i = 0; i < n; ++i) put(s[i]);
            return n;
        }
    private:
        void put(char c) {  // gọi khi đã giữ khóa
            if (c == '\n') {
                if (!line_.empty()) owner_.pushLocked(line_, warn_);
                line_.clear();
            } else if (c != '\r' && line_.size() < 400) line_ += c;
        }
        TuiRuntime& owner_;
        bool warn_;
        std::string line_;
    };

    TuiRuntime() : out_(*this, false), err_(*this, true) {}
    ~TuiRuntime() { stop(); }
    bool active() const { return active_; }
    // Bật khi stdout và stderr đều là terminal; trả về false (giữ in dòng như cũ) nếu không.
    bool start(const TuiState& initial) {
        if (active_ || !::isatty(STDOUT_FILENO) || !::isatty(STDERR_FILENO)) return false;
        std::cout.flush(); std::cerr.flush();
        { std::lock_guard<std::mutex> lock(mutex_); state_ = initial; }
        oldOut_ = std::cout.rdbuf(&out_);
        oldErr_ = std::cerr.rdbuf(&err_);
        active_ = true; quit_ = false;
        thread_ = std::thread([this] { loop(); });
        return true;
    }
    // Trả luồng xuất về terminal; dòng cuối giữ trên màn hình. Không gọi trong signal handler/_exit.
    void stop() {
        if (!active_) return;
        quit_ = true;
        if (thread_.joinable()) thread_.join();
        std::cout.rdbuf(oldOut_); std::cerr.rdbuf(oldErr_);
        active_ = false;
        draw(true);
        const char tail[] = "\033[0m\n";
        writeAll(tail, sizeof(tail) - 1, true);
    }
    // Vòng điều khiển gọi tối đa ~10 lần/giây: chép trạng thái, giữ nguyên danh sách sự kiện.
    template <class F> void update(F&& fill) {
        if (!active_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        fill(state_);
    }
private:
    void pushLocked(const std::string& text, bool warn) {
        state_.events.push_back({text, warn});
        while (state_.events.size() > size_t(kTuiEvents)) state_.events.pop_front();
    }
    static bool writeAll(const char* p, size_t n, bool block) {
        while (n) {
            pollfd fd{STDOUT_FILENO, POLLOUT, 0};
            if (::poll(&fd, 1, block ? 200 : 0) <= 0 || !(fd.revents & POLLOUT)) return false;
            const ssize_t k = ::write(STDOUT_FILENO, p, n < 256 ? n : 256);  // mẩu nhỏ: POLLOUT chỉ bảo đảm còn ít chỗ
            if (k <= 0) return false;
            p += k; n -= size_t(k);
        }
        return true;
    }
    void draw(bool block) {
        std::string frame;
        { std::lock_guard<std::mutex> lock(mutex_); frame = renderTui(state_, true); }
        writeAll(frame.data(), frame.size(), block);  // nghẽn: bỏ khung (không chặn)
    }
    void loop() {
        const char clear[] = "\033[H\033[J";
        writeAll(clear, sizeof(clear) - 1, false);
        while (!quit_) {
            draw(false);
            for (int i = 0; i < 10 && !quit_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::mutex mutex_;
    TuiState state_;
    LineBuf out_, err_;
    std::streambuf* oldOut_ = nullptr;
    std::streambuf* oldErr_ = nullptr;
    std::atomic<bool> quit_{false};
    bool active_ = false;
    std::thread thread_;
};
} // namespace auto_drive
