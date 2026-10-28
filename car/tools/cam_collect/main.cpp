#include "options.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {
volatile std::sig_atomic_t stopping = 0;
volatile std::sig_atomic_t watchdogExpired = 0;
void stop(int signal) { stopping = signal; }
void timeoutSignal(int) { watchdogExpired = 1; }

int64_t monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

int64_t wallMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string utcNow() {
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc{};
    gmtime_r(&t, &utc);
    std::ostringstream out;
    out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

struct WatchdogTimeout : std::runtime_error {
    WatchdogTimeout() : std::runtime_error("Quá hạn chờ khung hợp lệ") {}
};

int64_t timeoutMs() {
#if defined(UCR_CAM_TEST_HOOKS) && UCR_CAM_TEST_HOOKS
    if (const char* value = std::getenv("UCR_CAM_TEST_TIMEOUT_MS")) {
        const auto ms = number(value, "UCR_CAM_TEST_TIMEOUT_MS");
        if (ms == 0 || ms > 2147483647) throw std::runtime_error("Hạn thử không hợp lệ");
        return static_cast<int64_t>(ms);
    }
#endif
    return 5000;
}

// Tiến trình riêng vì OpenCV 4.1 không có tham số timeout cho open/read GStreamer.
struct Header { int status, width, height; int64_t monotonicNs, wallMs; };

bool sendAll(int fd, const void* data, size_t size) {
    const auto* p = static_cast<const char*>(data);
    while (size && !stopping) {
        const auto n = ::send(fd, p, size, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        p += n; size -= static_cast<size_t>(n);
    }
    return size == 0;
}

void captureWorker(int fd, const Options& o) {
    cv::VideoCapture cap;
    try {
#if defined(UCR_CAM_TEST_HOOKS) && UCR_CAM_TEST_HOOKS
        const char* testSource = std::getenv("UCR_CAM_TEST_SOURCE");
        const std::string fake = testSource ? testSource : "";
        const bool synthetic = fake == "never" || fake.compare(0, 12, "stall_after:") == 0;
        const uint64_t fakeFrames = fake == "never" ? 0 :
            (synthetic ? number(fake.substr(12), "UCR_CAM_TEST_SOURCE") : 0);
        uint64_t produced = 0;
        const bool opened = synthetic ||
#else
        const bool opened =
#endif
            (o.source.empty() ? cap.open(o.pipeline, cv::CAP_GSTREAMER) : cap.open(o.source.string()));
        if (!opened) { Header h{}; h.status = -2; sendAll(fd, &h, sizeof(h)); return; }
        char request;
        while (!stopping && ::recv(fd, &request, 1, 0) == 1) {
            cv::Mat frame;
            bool ok;
#if defined(UCR_CAM_TEST_HOOKS) && UCR_CAM_TEST_HOOKS
            if (synthetic) {
                if (produced == fakeFrames) {
                    while (!stopping) std::this_thread::sleep_for(10ms);
                    break;
                }
                frame = cv::Mat(384, 640, CV_8UC3, cv::Scalar(80, 120, 160));
                ++produced;
                std::this_thread::sleep_for(10ms);
                ok = true;
            } else
#endif
            ok = cap.read(frame) && !frame.empty();
            Header h{};
            h.status = ok ? 1 : (o.source.empty() ? 0 : -1);
            h.monotonicNs = monotonicNs();
            h.wallMs = wallMs();
            if (ok) {
                if (frame.type() != CV_8UC3) throw std::runtime_error("Khung phải là BGR 8 bit, 3 kênh");
                if (!frame.isContinuous()) frame = frame.clone();
                h.width = frame.cols; h.height = frame.rows;
            }
            if (!sendAll(fd, &h, sizeof(h))) break;
            if (ok && !sendAll(fd, frame.data, frame.total() * frame.elemSize())) break;
            if (h.status == -1) break;
            if (!ok) std::this_thread::sleep_for(10ms);
        }
    } catch (const std::exception& e) {
        std::cerr << "Lỗi đọc nguồn: " << e.what() << '\n';
        Header h{}; h.status = -2; sendAll(fd, &h, sizeof(h));
    }
    cap.release();
}

class Capture {
    int fd_ = -1;
    int heartbeat_ = -1;
    pid_t pid_ = -1, watchdog_ = -1;
    Clock::time_point lastValid_, end_;
    std::chrono::milliseconds timeout_;
public:
    Capture(const Options& o, Clock::time_point started, Clock::time_point end, int64_t timeout)
        : lastValid_(started), end_(end), timeout_(timeout) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) throw std::runtime_error("Không tạo được kênh đọc camera");
        pid_ = fork();
        if (pid_ == 0) {
            ::close(sockets[0]);
            try { captureWorker(sockets[1], o); }
            catch (...) { ::close(sockets[1]); _exit(1); }
            ::close(sockets[1]);
            _exit(0);
        }
        ::close(sockets[1]);
        if (pid_ < 0) { ::close(sockets[0]); throw std::runtime_error("Không tạo được tiến trình đọc camera"); }
        fd_ = sockets[0];
        int beats[2];
        if (pipe(beats) != 0) {
            kill(pid_, SIGKILL); waitpid(pid_, nullptr, 0); ::close(fd_);
            throw std::runtime_error("Không tạo được kênh giám sát");
        }
        if (fcntl(beats[0], F_SETFL, O_NONBLOCK) < 0) {
            ::close(beats[0]); ::close(beats[1]); ::close(fd_);
            kill(pid_, SIGKILL); waitpid(pid_, nullptr, 0);
            throw std::runtime_error("Không đặt được kênh giám sát không chặn");
        }
        const pid_t parent = getpid();
        watchdog_ = fork();
        if (watchdog_ == 0) {
            ::close(beats[1]); ::close(fd_);
            // Giám sát sống độc lập với open/read/release của OpenCV.
            signal(SIGINT, SIG_IGN); signal(SIGTERM, SIG_IGN); signal(SIGHUP, SIG_IGN);
            int64_t last = std::chrono::duration_cast<std::chrono::nanoseconds>(started.time_since_epoch()).count();
            while (true) {
                pollfd ready{beats[0], POLLIN, 0};
                const int result = poll(&ready, 1, 20);
                if (result < 0 && errno == EINTR) continue;
                if (result < 0) _exit(1);
                if (result > 0) {
                    // Đọc hết nhịp còn đọng trước khi xét hạn.
                    while (true) {
                        int64_t beat;
                        const auto n = ::read(beats[0], &beat, sizeof(beat));
                        if (n < 0 && errno == EINTR) continue;
                        if (n < 0 && errno == EAGAIN) break;
                        if (n != sizeof(beat)) _exit(0);
                        last = beat;
                    }
                }
                if (monotonicNs() - last >= timeout * 1000000LL) {
                    kill(pid_, SIGKILL);
                    kill(parent, SIGUSR1);
                    _exit(0);
                }
            }
        }
        ::close(beats[0]);
        if (watchdog_ < 0) {
            ::close(beats[1]); ::close(fd_); kill(pid_, SIGKILL); waitpid(pid_, nullptr, 0);
            throw std::runtime_error("Không tạo được tiến trình giám sát");
        }
        heartbeat_ = beats[1];
    }
    ~Capture() {
        if (heartbeat_ >= 0) ::close(heartbeat_);
        if (watchdog_ > 0) {
            kill(watchdog_, SIGKILL);
            while (waitpid(watchdog_, nullptr, 0) < 0 && errno == EINTR) {}
        }
        if (fd_ >= 0) { shutdown(fd_, SHUT_RDWR); ::close(fd_); }
        if (pid_ <= 0) return;
        kill(pid_, SIGTERM);
        for (int i = 0; i < 25; ++i) {
            if (waitpid(pid_, nullptr, WNOHANG) == pid_) return;
            std::this_thread::sleep_for(20ms);
        }
        // Chỉ cưỡng bức khi thư viện camera không chịu trả quyền điều khiển.
        kill(pid_, SIGKILL);
        while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
    }
    bool receive(void* data, size_t size) {
        auto* p = static_cast<char*>(data);
        while (size && !stopping) {
            const auto now = Clock::now();
            if (now >= end_ && end_ <= lastValid_ + timeout_) return false;
            if (watchdogExpired || now >= lastValid_ + timeout_) throw WatchdogTimeout();
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::min(lastValid_ + timeout_, end_) - Clock::now()).count();
            pollfd ready{fd_, POLLIN, 0};
            const int result = poll(&ready, 1, static_cast<int>(std::max<int64_t>(0, std::min<int64_t>(left, 100))));
            if (result < 0 && errno == EINTR) continue;
            if (result < 0) throw std::runtime_error("Không chờ được dữ liệu camera");
            if (result == 0) continue;
            const auto n = recv(fd_, p, size, MSG_DONTWAIT);
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) {
                if (stopping) return false;
                if (watchdogExpired || Clock::now() >= lastValid_ + timeout_) throw WatchdogTimeout();
                throw std::runtime_error("Tiến trình đọc nguồn đã đóng kết nối");
            }
            p += n; size -= static_cast<size_t>(n);
        }
        return size == 0;
    }
    bool read(cv::Mat& frame, Header& h) {
        if (watchdogExpired) throw WatchdogTimeout();
        const char request = 1;
        if (!sendAll(fd_, &request, 1)) {
            if (stopping) return false;
            // Tiến trình đọc có thể đã báo lỗi mở nguồn rồi đóng kênh; đọc lý do còn trong bộ đệm.
            if (receive(&h, sizeof(h)) && h.status == -2) throw std::runtime_error("Không mở/đọc được nguồn");
            throw std::runtime_error("Không gửi được yêu cầu đọc nguồn");
        }
        if (!receive(&h, sizeof(h))) return false;
        if (h.status == -2) throw std::runtime_error("Không mở/đọc được nguồn");
        if (h.status != 1) return true;
        if (h.width <= 0 || h.height <= 0) throw std::runtime_error("Kích thước khung không hợp lệ");
        frame.create(h.height, h.width, CV_8UC3);
        if (!receive(frame.data, frame.total() * frame.elemSize())) return false;
        lastValid_ = Clock::now();
        const int64_t beat = monotonicNs();
        ssize_t sent;
        do { sent = ::write(heartbeat_, &beat, sizeof(beat)); } while (sent < 0 && errno == EINTR && !stopping);
        if (sent != sizeof(beat)) {
            if (stopping) return false;
            throw WatchdogTimeout();
        }
        return true;
    }
};

double freeMB(const fs::path& path) {
    struct statvfs info{};
    if (statvfs(path.c_str(), &info) != 0) throw std::runtime_error("Không đọc được dung lượng ổ đĩa");
    return static_cast<double>(info.f_bavail) * info.f_frsize / (1024.0 * 1024.0);
}

int exitCode(const std::string& reason) {
    if (reason == "no_frame_timeout") return 3;
    if (reason == "stall_timeout") return 4;
    if (reason == "disk_low") return 5;
    return reason == "error" ? 1 : 0;
}

std::string signalReason() {
    if (stopping == SIGINT) return "sigint";
    if (stopping == SIGHUP) return "sighup";
    return "sigterm";
}

void writeSession(const Options& o, const std::string& started, const std::string& ended) {
    char hostname[256]{};
    if (gethostname(hostname, sizeof(hostname) - 1) != 0) hostname[0] = 0;
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    const auto temp = o.out / "session.json.tmp";
    file.open(temp.string());
    file << "{\n\"label\":" << jsonString(o.label) << ",\n\"who\":" << jsonString(o.who)
         << ",\n\"weather\":" << jsonString(o.weather) << ",\n\"curtain\":" << jsonString(o.curtain)
         << ",\n\"lights\":" << jsonString(o.lights) << ",\n\"props\":" << jsonString(o.props)
         << ",\n\"note\":" << jsonString(o.note) << ",\n\"started_utc\":" << jsonString(started)
         << ",\n\"ended_utc\":" << jsonString(ended) << ",\n\"hostname\":" << jsonString(hostname)
         << ",\n\"pipeline_used\":" << jsonString(o.source.empty() ? o.pipeline : "source:" + o.source.string())
         << ",\n\"version\":\"ucr_collect_cam/1\",\n\"params\":{\"interval_ms\":" << o.interval
         << ",\"max_frames\":" << o.maxFrames << ",\"max_seconds\":" << o.maxSeconds
         << ",\"warmup_frames\":" << o.warmup << ",\"jpeg_quality\":" << o.quality
         << ",\"min_free_mb\":" << o.minFree << ",\"start_delay_s\":" << o.startDelay << "}\n}\n";
    file.close();
    fs::rename(temp, o.out / "session.json");
}

// BT.601 và Laplacian 4 láng giềng, biên phản chiếu; không cần thêm thư viện imgproc.
std::pair<double, double> imageQuality(const cv::Mat& frame) {
    cv::Mat gray(frame.rows, frame.cols, CV_8UC1);
    double sum = 0, lapSum = 0, lapSquared = 0;
    for (int y = 0; y < frame.rows; ++y) {
        const auto* row = frame.ptr<cv::Vec3b>(y);
        auto* target = gray.ptr<unsigned char>(y);
        for (int x = 0; x < frame.cols; ++x) {
            target[x] = static_cast<unsigned char>((row[x][0] * 1868 + row[x][1] * 9617 + row[x][2] * 4899 + 8192) >> 14);
            sum += target[x];
        }
    }
    for (int y = 0; y < frame.rows; ++y) {
        const auto* row = gray.ptr<unsigned char>(y);
        const auto* above = gray.ptr<unsigned char>(y > 0 ? y - 1 : std::min(1, frame.rows - 1));
        const auto* below = gray.ptr<unsigned char>(y + 1 < frame.rows ? y + 1 : std::max(0, y - 1));
        for (int x = 0; x < frame.cols; ++x) {
            const int left = x > 0 ? x - 1 : std::min(1, frame.cols - 1);
            const int right = x + 1 < frame.cols ? x + 1 : std::max(0, x - 1);
            const double lap = above[x] + below[x] + row[left] + row[right] - 4 * row[x];
            lapSum += lap; lapSquared += lap * lap;
        }
    }
    const double pixels = static_cast<double>(frame.total());
    return {sum / pixels, std::max(0.0, lapSquared / pixels - (lapSum / pixels) * (lapSum / pixels))};
}

struct Stats {
    uint64_t saved = 0, read = 0, empty = 0, bytes = 0, warmup = 0;
    int width = 0, height = 0;
    double freeStart = 0, freeEnd = 0;
    std::string reason = "error";
    std::map<std::pair<int, int>, uint64_t> sizes;
};

void writeSummary(const Options& o, const Stats& s, double elapsed) {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open((o.out / "summary.json.tmp").string());
    const double saveFps = elapsed > 0 ? s.saved / elapsed : 0;
    file << std::setprecision(12) << "{\n\"total_images\":" << s.saved
         << ",\n\"average_fps\":" << saveFps << ",\n\"duration_seconds\":" << elapsed
         << ",\n\"empty_frames\":" << s.empty << ",\n\"width\":" << s.width << ",\n\"height\":" << s.height
         << ",\n\"exit_code\":" << exitCode(s.reason) << ",\n\"exit_reason\":" << jsonString(s.reason)
         << ",\n\"read_fps\":" << (elapsed > 0 ? s.read / elapsed : 0) << ",\n\"save_fps\":" << saveFps
         << ",\n\"total_bytes\":" << s.bytes << ",\n\"warmup_dropped\":" << s.warmup
         << ",\n\"free_mb_start\":" << s.freeStart << ",\n\"free_mb_end\":" << s.freeEnd << ",\n\"frame_sizes\":[";
    bool first = true;
    for (const auto& size : s.sizes) {
        if (!first) file << ',';
        first = false;
        file << "{\"width\":" << size.first.first << ",\"height\":" << size.first.second << ",\"count\":" << size.second << '}';
    }
    file << "]\n}\n";
    file.close();
    fs::rename(o.out / "summary.json.tmp", o.out / "summary.json");
}

int collect(Options o, Clock::time_point started, const std::string& startedUtc) {
    fs::create_directories(o.out.parent_path().empty() ? fs::path(".") : o.out.parent_path());
    const auto base = o.out;
    for (uint64_t suffix = 2; ; ++suffix) {
        std::error_code error;
        if (fs::create_directory(o.out, error)) break;
        if (o.explicitOut || !fs::exists(o.out))
            throw std::runtime_error("Không tạo được thư mục đầu ra mới: " + o.out.string() + ": " + error.message());
        o.out = base.string() + '_' + std::to_string(suffix);
    }
    Stats stats;
    std::ofstream csv;
    const auto end = o.maxSeconds ? started + std::chrono::seconds(o.maxSeconds) : Clock::time_point::max();
    const auto saveAfter = started + std::chrono::seconds(o.startDelay);
    auto nextSave = saveAfter;
    auto nextDisk = started;
    auto nextProgress = started + 10s;
    uint64_t countdown = 0, dropped = 0;
    bool warned = false;
    // Kiểm cả lúc chờ lưu để tín hiệu và hạn chạy luôn có hiệu lực.
    const auto tick = [&]() {
        const auto now = Clock::now();
        if (stopping) { stats.reason = signalReason(); return false; }
        if (now >= end) { stats.reason = "max_seconds"; return false; }
        if (watchdogExpired) {
            stats.reason = stats.read ? "stall_timeout" : "no_frame_timeout";
            std::cerr << "Lỗi: Quá hạn chờ khung hợp lệ\nNguồn: " << (o.source.empty() ? o.pipeline : o.source.string())
                      << "\nCamera đang bị chương trình khác dùng? (pgrep -a main)\n";
            return false;
        }
        if (now >= nextDisk) {
            stats.freeEnd = freeMB(o.out);
            nextDisk = now + 1s;
            if (stats.freeEnd < o.minFree) {
                std::cerr << "Không đủ chỗ trống: " << stats.freeEnd << " MiB, cần " << o.minFree << " MiB\n";
                stats.reason = "disk_low";
                return false;
            }
            if (stats.freeEnd < 2.0L * o.minFree && !warned) {
                std::cerr << "Cảnh báo: dung lượng dưới hai lần mức dự phòng\n";
                warned = true;
            }
        }
        if (now >= nextProgress) {
            const double elapsed = std::chrono::duration<double>(now - started).count();
            std::cerr << "tien do: saved=" << stats.saved << " read_fps=" << stats.read / elapsed
                      << " save_fps=" << stats.saved / elapsed << " free_mb=" << stats.freeEnd << '\n';
            nextProgress = now + 10s;
        }
        if (now < saveAfter) {
            const auto left = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(saveAfter - now).count()) + 1;
            if (left != countdown) { std::cerr << "Đếm lùi: " << left << " s\n"; countdown = left; }
        }
        return true;
    };
    try {
        stats.freeStart = stats.freeEnd = freeMB(o.out);
        writeSession(o, startedUtc, "");
        csv.exceptions(std::ios::badbit | std::ios::failbit);
        csv.open((o.out / "metadata.csv").string());
        csv << "frame,monotonic_ns,wall_time_unix_ms,filename,width,height,mean_luma,lap_var,dropped_since_last\n" << std::flush;
        if (tick()) {
            std::vector<fs::path> images;
            const bool directory = !o.source.empty() && fs::is_directory(o.source);
            if (directory) {
                for (const auto& entry : fs::directory_iterator(o.source)) {
                    auto ext = entry.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    if (fs::is_regular_file(entry.path()) && (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".ppm"))
                        images.push_back(entry.path());
                }
                std::sort(images.begin(), images.end());
            }
            std::unique_ptr<Capture> capture;
            if (!directory) capture.reset(new Capture(o, started, end, timeoutMs()));
            size_t index = 0;
            while (tick()) {
                if (o.maxFrames && stats.saved >= o.maxFrames) { stats.reason = "max_frames"; break; }
                cv::Mat frame;
                Header h{};
                if (directory) {
                    if (index == images.size()) { stats.reason = "eof"; break; }
                    while (Clock::now() < nextSave && tick()) std::this_thread::sleep_for(10ms);
                    if (!tick()) break;
                    const auto path = images[index++];
                    frame = cv::imread(path.string(), cv::IMREAD_COLOR);
                    if (frame.empty()) { ++stats.empty; throw std::runtime_error("Không đọc được ảnh: " + path.string()); }
                    h.monotonicNs = monotonicNs(); h.wallMs = wallMs();
                } else {
                    if (!capture->read(frame, h)) {
                        stats.reason = stopping ? signalReason() : "max_seconds";
                        break;
                    }
                    if (h.status == -1) { stats.reason = "eof"; break; }
                    if (h.status == 0) { ++stats.empty; continue; }
                }
                ++stats.read;
                if (!directory && stats.warmup < o.warmup) { ++stats.warmup; continue; }
                if (!tick()) break;
                if (Clock::now() < nextSave) { ++dropped; continue; }
                const auto quality = imageQuality(frame);
                std::ostringstream filename;
                filename << "frame_" << std::setw(8) << std::setfill('0') << stats.saved + 1 << ".jpg";
                const auto target = o.out / filename.str();
                const fs::path temp = target.string() + ".tmp";
                std::vector<unsigned char> encoded;
                if (!cv::imencode(".jpg", frame, encoded, {cv::IMWRITE_JPEG_QUALITY, o.quality}))
                    throw std::runtime_error("Không mã hoá được ảnh JPEG");
                std::ofstream image;
                image.exceptions(std::ios::badbit | std::ios::failbit);
                image.open(temp.string(), std::ios::binary);
                image.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
                image.close();
                fs::rename(temp, target);
                stats.width = frame.cols; stats.height = frame.rows;
                ++stats.saved; stats.bytes += encoded.size();
                ++stats.sizes[{stats.width, stats.height}];
                csv << std::setprecision(12) << stats.saved << ',' << h.monotonicNs << ',' << h.wallMs << ',' << filename.str()
                    << ',' << stats.width << ',' << stats.height << ',' << quality.first << ',' << quality.second << ',' << dropped << '\n' << std::flush;
                dropped = 0;
                nextSave = Clock::now() + std::chrono::milliseconds(o.interval);
            }
        }
    } catch (const WatchdogTimeout& e) {
        std::cerr << "Lỗi: " << e.what() << "\nNguồn: " << (o.source.empty() ? o.pipeline : o.source.string())
                  << "\nCamera đang bị chương trình khác dùng? (pgrep -a main)\n";
        stats.reason = stats.read ? "stall_timeout" : "no_frame_timeout";
    } catch (const std::exception& e) {
        std::cerr << "Lỗi: " << e.what() << "\nNguồn: " << (o.source.empty() ? o.pipeline : o.source.string())
                  << "\nCamera đang bị chương trình khác dùng? (pgrep -a main)\n";
        stats.reason = "error";
    }
    if (stopping) stats.reason = signalReason();
    try { if (csv.is_open()) { csv.flush(); csv.close(); } }
    catch (const std::exception& e) { std::cerr << "Lỗi đóng CSV: " << e.what() << '\n'; stats.reason = "error"; }
    try { stats.freeEnd = freeMB(o.out); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; stats.reason = "error"; }
    try { writeSession(o, startedUtc, utcNow()); }
    catch (const std::exception& e) { std::cerr << "Lỗi ghi phiên: " << e.what() << '\n'; stats.reason = "error"; }
    try { writeSummary(o, stats, std::chrono::duration<double>(Clock::now() - started).count()); }
    catch (const std::exception& e) { std::cerr << "Lỗi ghi tổng kết: " << e.what() << '\n'; return 1; }
    std::cout << "Đã lưu " << stats.saved << " ảnh tại " << o.out << '\n';
    return exitCode(stats.reason);
}
}

int main(int argc, char** argv) {
    const auto started = Clock::now();
    const auto startedUtc = utcNow();
    struct sigaction action{};
    action.sa_handler = stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    sigaction(SIGHUP, &action, nullptr);
    action.sa_handler = timeoutSignal;
    sigaction(SIGUSR1, &action, nullptr);
    signal(SIGPIPE, SIG_IGN);
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "ucr_collect_cam --help | [--pipeline CHUỖI | --source VIDEO_HOẶC_THƯ_MỤC]\n"
                  << "  --out-dir THƯ_MỤC_MỚI (ưu tiên hơn --out-root)\n"
                  << "  --out-root runs_cam --label cam --who CHUỖI --weather CHUỖI\n"
                  << "  --curtain CHUỖI --lights CHUỖI --props CHUỖI --note CHUỖI\n"
                  << "  --interval-ms 1000 --max-frames 0 --max-seconds 0\n"
                  << "  --start-delay-s 0 --warmup-frames 15 --jpeg-quality 90 --min-free-mb 1024\n"
                  << "Lưu nguyên cỡ nguồn; 0 ở hai giới hạn: không giới hạn.\n"
                  << "Thư mục nguồn bỏ qua warmup; đếm lùi trước khi lưu, vẫn đọc camera.\n";
        return 0;
    }
    Options options;
    try { options = parse(argc, argv); }
    catch (const std::exception& e) { std::cerr << "Lỗi tham số: " << e.what() << '\n'; return 2; }
    try { return collect(options, started, startedUtc); }
    catch (const std::exception& e) { std::cerr << "Lỗi: " << e.what() << '\n'; return 1; }
}
