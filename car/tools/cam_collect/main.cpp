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
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

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
        const bool opened = o.source.empty() ? cap.open(o.pipeline, cv::CAP_GSTREAMER) : cap.open(o.source.string());
        if (!opened) { Header h{}; h.status = -2; sendAll(fd, &h, sizeof(h)); return; }
        char request;
        while (!stopping && ::recv(fd, &request, 1, 0) == 1) {
            cv::Mat frame;
            const bool ok = cap.read(frame) && !frame.empty();
            Header h{};
            h.status = ok ? 1 : (o.source.empty() ? 0 : -1);
            h.monotonicNs = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
            h.wallMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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
    pid_t pid_ = -1;
public:
    explicit Capture(const Options& o) {
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
    }
    ~Capture() {
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
    bool receive(void* data, size_t size, Clock::time_point deadline) {
        auto* p = static_cast<char*>(data);
        while (size && !stopping) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
            if (left <= 0) throw std::runtime_error("Không có khung hợp lệ trong 5 giây");
            pollfd ready{fd_, POLLIN, 0};
            const int result = poll(&ready, 1, static_cast<int>(std::min<int64_t>(left, 100)));
            if (result < 0 && errno == EINTR) continue;
            if (result < 0) throw std::runtime_error("Không chờ được dữ liệu camera");
            if (result == 0) continue;
            const auto n = recv(fd_, p, size, MSG_DONTWAIT);
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) {
                if (stopping) return false;
                throw std::runtime_error("Tiến trình đọc nguồn đã đóng kết nối");
            }
            p += n; size -= static_cast<size_t>(n);
        }
        return size == 0;
    }
    bool read(cv::Mat& frame, Header& h, Clock::time_point deadline) {
        const char request = 1;
        if (!sendAll(fd_, &request, 1)) {
            if (stopping) return false;
            throw std::runtime_error("Không gửi được yêu cầu đọc nguồn");
        }
        if (!receive(&h, sizeof(h), deadline)) return false;
        if (h.status == -2) throw std::runtime_error("Không mở/đọc được nguồn");
        if (h.status != 1) return true;
        if (h.width <= 0 || h.height <= 0) throw std::runtime_error("Kích thước khung không hợp lệ");
        frame.create(h.height, h.width, CV_8UC3);
        return receive(frame.data, frame.total() * frame.elemSize(), deadline);
    }
};

bool diskSpace(const Options& o, bool& warned) {
    const auto mb = fs::space(o.out).available / (1024ULL * 1024ULL);
    if (mb < o.minFree) {
        std::cerr << "Không đủ chỗ trống: " << mb << " MiB, cần ít nhất " << o.minFree << " MiB\n";
        return false;
    }
    if (static_cast<long double>(mb) < 2.0L * o.minFree && !warned) {
        std::cerr << "Cảnh báo: còn " << mb << " MiB, dưới hai lần mức dự phòng\n";
        warned = true;
    }
    return true;
}

int collect(const Options& o) {
    if (fs::exists(o.out)) throw std::runtime_error("Thư mục đầu ra đã tồn tại, không ghi đè: " + o.out.string());
    fs::create_directories(o.out.parent_path().empty() ? fs::path(".") : o.out.parent_path());
    if (!fs::create_directory(o.out)) throw std::runtime_error("Không tạo được thư mục đầu ra");
    bool warned = false;
    if (!diskSpace(o, warned)) return 1;
    const auto started = Clock::now();
    uint64_t saved = 0, empty = 0;
    int width = 0, height = 0, result = 0;
    std::map<std::pair<int, int>, uint64_t> sizes;
    std::ofstream csv((o.out / "metadata.csv").string());
    csv.exceptions(std::ios::badbit | std::ios::failbit);
    csv << "frame,monotonic_ns,wall_time_unix_ms,filename,width,height\n" << std::flush;
    try {
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
            if (images.empty()) throw std::runtime_error("Thư mục nguồn không có ảnh được hỗ trợ");
        }
        // Không mở camera/video khi nguồn là thư mục ảnh.
        std::unique_ptr<Capture> capture;
        if (!directory && !stopping) capture.reset(new Capture(o));
        size_t index = 0;
        auto lastValid = Clock::now();
        auto nextSave = started;
        auto lastSaved = started;
        while (!stopping && (o.maxFrames == 0 || saved < o.maxFrames)) {
            if (!diskSpace(o, warned)) { result = 1; break; }
            cv::Mat frame;
            Header h{};
            if (!o.source.empty()) {
                if (directory && index == images.size()) break;
                while (!stopping && Clock::now() < nextSave) {
                    if (Clock::now() - lastSaved >= 1s) {
                        if (!diskSpace(o, warned)) { result = 1; break; }
                        lastSaved = Clock::now();
                    }
                    std::this_thread::sleep_for(10ms);
                }
                if (stopping || result) break;
            }
            if (directory) {
                const auto path = images[index++];
                frame = cv::imread(path.string(), cv::IMREAD_COLOR);
                if (frame.empty()) { ++empty; throw std::runtime_error("Không đọc được ảnh: " + path.string()); }
                h.monotonicNs = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
                h.wallMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            } else {
                if (!capture->read(frame, h, o.source.empty() ? lastValid + 5s : Clock::now() + 5s)) break;
                if (h.status == -1) {
                    if (saved == 0) throw std::runtime_error("Nguồn kết thúc trước khi lưu được ảnh");
                    break;
                }
                if (h.status == 0) { ++empty; continue; }
                lastValid = Clock::now();
                if (Clock::now() < nextSave) continue;
            }
            if (stopping) break;
            if (!diskSpace(o, warned)) { result = 1; break; }
            std::ostringstream filename;
            filename << "frame_" << std::setw(8) << std::setfill('0') << saved + 1 << ".jpg";
            if (!cv::imwrite((o.out / filename.str()).string(), frame, {cv::IMWRITE_JPEG_QUALITY, o.quality}))
                throw std::runtime_error("Không ghi được ảnh JPEG");
            width = frame.cols; height = frame.rows;
            ++saved;
            ++sizes[{width, height}];
            csv << saved << ',' << h.monotonicNs << ',' << h.wallMs << ',' << filename.str() << ',' << width << ',' << height << '\n';
            csv.flush();
            lastSaved = Clock::now();
            nextSave = lastSaved + std::chrono::milliseconds(o.interval);
        }
    } catch (const std::exception& e) {
        std::cerr << "Lỗi: " << e.what() << "\nNguồn: " << (o.source.empty() ? o.pipeline : o.source.string()) << '\n';
        result = 1;
    }
    try { csv.flush(); csv.close(); }
    catch (const std::exception& e) { std::cerr << "Lỗi đóng CSV: " << e.what() << '\n'; result = 1; }
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    std::ofstream summary((o.out / "summary.json").string());
    summary.exceptions(std::ios::badbit | std::ios::failbit);
    summary << std::setprecision(12) << "{\n  \"total_images\": " << saved
            << ",\n  \"average_fps\": " << (elapsed > 0 ? saved / elapsed : 0)
            << ",\n  \"duration_seconds\": " << elapsed << ",\n  \"empty_frames\": " << empty
            << ",\n  \"width\": " << width << ",\n  \"height\": " << height
            << ",\n  \"exit_code\": " << result << ",\n  \"frame_sizes\": [";
    bool first = true;
    for (const auto& size : sizes) {
        if (!first) summary << ',';
        first = false;
        summary << "{\"width\":" << size.first.first << ",\"height\":" << size.first.second << ",\"count\":" << size.second << '}';
    }
    summary << "]\n}\n";
    summary.close();
    std::cout << "Đã lưu " << saved << " ảnh tại " << o.out << '\n';
    return result;
}
}

int main(int argc, char** argv) {
    struct sigaction action{};
    action.sa_handler = stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "ucr_collect_cam [--pipeline CHUỖI | --source VIDEO_HOẶC_THƯ_MỤC]\n"
                  << "  --out-dir THƯ_MỤC_MỚI (mặc định runs/cam_<giờ>_<pid>)\n"
                  << "  --interval-ms 200 --max-frames 0 --jpeg-quality 90 --min-free-mb 1024\n"
                  << "Lưu nguyên kích thước nguồn; --max-frames 0: đến khi Ctrl+C hoặc hết nguồn.\n";
        return 0;
    }
    try { return collect(parse(argc, argv)); }
    catch (const std::exception& e) { std::cerr << "Lỗi: " << e.what() << '\n'; return 1; }
}
