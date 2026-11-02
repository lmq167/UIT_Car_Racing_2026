// Luồng model: không chặn bên gửi, không lệch khung, lỗi chỉ thành thông báo, dừng có hạn.
#include "MaskRender.h"
#include "ModelWorker.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef UCR_TSAN_SUPP
// Như harness T33: race giả trong OpenCV/TBB chưa instrument, xem tools/drive_collect/tsan.supp.
extern "C" const char* __tsan_default_options() { return "suppressions=" UCR_TSAN_SUPP; }
#endif

namespace {
using Clock = std::chrono::steady_clock;
int failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) { std::cerr << "HONG: " << what << '\n'; ++failures; }
}
int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
model_view::ModelInfo fakeInfo() {
    model_view::ModelInfo info;
    info.name = "fake";
    info.classNames = {"background", "track_surface", "line"};
    info.sourceH = 720; info.sourceW = 1280; info.cropY0 = 320; info.cropY1 = 720; info.cropX1 = 1280;
    info.inputH = 96; info.inputW = 160;
    return info;
}
// Lớp của mặt nạ suy ra từ chính khung được suy luận (độ xám v → lớp v % 3).
class FrameModel : public model_view::IMaskModel {
public:
    FrameModel(int sleepMs, bool fail) : sleepMs_(sleepMs), fail_(fail) {}
    const model_view::ModelInfo& info() const override { return info_; }
    cv::Mat infer(const cv::Mat& bgr) override {
        const int v = bgr.at<cv::Vec3b>(0, 0)[0];
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs_));
        if (fail_) throw std::runtime_error("loi suy luan gia");
        return cv::Mat(info_.inputH, info_.inputW, CV_8UC1, cv::Scalar(v % 3));
    }
private:
    model_view::ModelInfo info_ = fakeInfo();
    int sleepMs_;
    bool fail_;
};
struct Collector {
    std::mutex mutex;
    std::vector<model_view::ModelPanels> panels;
    void operator()(const model_view::ModelPanels& p) { std::lock_guard<std::mutex> lock(mutex); panels.push_back(p); }
    size_t size() { std::lock_guard<std::mutex> lock(mutex); return panels.size(); }
};
cv::Mat frameOf(int v) { return cv::Mat(720, 1280, CV_8UC3, cv::Scalar(v, v, v)); }

struct LatestSamples {
    std::mutex mutex;
    int64_t offered[200] = {};
    std::vector<int64_t> ages;
};
class LatestModel : public model_view::IMaskModel {
public:
    explicit LatestModel(LatestSamples& samples) : samples_(samples) {}
    const model_view::ModelInfo& info() const override { return info_; }
    cv::Mat infer(const cv::Mat& bgr) override {
        const auto started = nowMs();
        const int id = bgr.at<cv::Vec3b>(0, 0)[0];
        {
            std::lock_guard<std::mutex> lock(samples_.mutex);
            samples_.ages.push_back(started - samples_.offered[id]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return cv::Mat(info_.inputH, info_.inputW, CV_8UC1, cv::Scalar(id % 3));
    }
private:
    LatestSamples& samples_;
    model_view::ModelInfo info_ = fakeInfo();
};

void latestFrameCase() {
    LatestSamples samples;
    Collector out;
    {
        model_view::ModelWorker worker([&] {
            return std::unique_ptr<model_view::IMaskModel>(new LatestModel(samples));
        }, std::ref(out), 15, "latest");
        auto next = Clock::now();
        for (int id = 1; id <= 100; ++id) {
            const cv::Mat frame = frameOf(id);
            const auto stamp = nowMs();
            { std::lock_guard<std::mutex> lock(samples.mutex); samples.offered[id] = stamp; }
            worker.offer(frame, stamp);
            next += std::chrono::milliseconds(20);
            std::this_thread::sleep_until(next);
        }
        worker.requestStop();
    }
    check(samples.ages.size() >= 4 && out.size() == samples.ages.size(), "latest_frame: moi luot deu co ket qua");
    // 20 ms chu kỳ offer + 130 ms dung sai lập lịch (TSan); FIFO giữ ảnh ít nhất một lượt suy luận 300 ms.
    for (const auto age : samples.ages)
        check(age >= 0 && age <= 150, "latest_frame: tuoi khung luc bat dau suy luan " + std::to_string(age) + " ms");
}

void frameMatchAndNonBlocking() {
    Collector out;
    double worstOfferMs = 0;
    {
        model_view::ModelWorker worker([] { return std::unique_ptr<model_view::IMaskModel>(new FrameModel(300, false)); },
                                       std::ref(out), 15, "test");
        const auto end = Clock::now() + std::chrono::milliseconds(2000);
        for (int i = 0; Clock::now() < end; ++i) {
            const cv::Mat frame = frameOf(30 + (i % 200));
            const auto start = Clock::now();
            worker.offer(frame, nowMs());
            worstOfferMs = std::max(worstOfferMs,
                                    std::chrono::duration<double, std::milli>(Clock::now() - start).count());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    // Ngưỡng nửa thời gian suy luận 300 ms: bản chờ model (M7) chắc chắn vượt; 1 CPU (container) có thể bị luồng model chen.
    check(worstOfferMs < 150, "offer khong cho model (cham nhat " + std::to_string(worstOfferMs) + " ms)");
    check(out.size() >= 3, "co ket qua khi model cham 300 ms");
    const cv::Vec3b colors[3] = {{0, 0, 0}, {0, 160, 0}, {0, 255, 255}};
    for (const auto& p : out.panels) {
        if (!p.notice.empty()) { check(false, "khong duoc co thong bao: " + p.notice); continue; }
        const int v = p.overlay.at<cv::Vec3b>(150, 600)[0];   // trên crop_y: ảnh gốc thu nhỏ
        const cv::Vec3b expect = colors[v % 3];
        const cv::Vec3b blended((expect[0] + v + 1) / 2, (expect[1] + v + 1) / 2, (expect[2] + v + 1) / 2);
        const cv::Vec3b got = p.overlay.at<cv::Vec3b>(300, 600);
        bool same = true;
        for (int c = 0; c < 3; ++c) same = same && std::abs(int(got[c]) - int(blended[c])) <= 1;
        check(same, "frame_mismatch: mat na ve len anh khac (v=" + std::to_string(v) + ")");
        check(p.mask.at<cv::Vec3b>(300, 600) == expect, "frame_mismatch: panel mat na khac lop cua khung");
        check(p.inferMs >= 290 && p.resultAgeMs >= p.inferMs - 1, "do tre va tuoi ket qua");
    }
}

void errorsBecomeNotices() {
    Collector failing, throwing, empty;
    {
        model_view::ModelWorker a([] { return std::unique_ptr<model_view::IMaskModel>(new FrameModel(0, true)); },
                                  std::ref(failing), 5, "");
        model_view::ModelWorker b([]() -> std::unique_ptr<model_view::IMaskModel> {
                                      throw std::runtime_error("onnx hong gia"); },
                                  std::ref(throwing), 5, "");
        model_view::ModelWorker c([] { return std::unique_ptr<model_view::IMaskModel>(); }, std::ref(empty), 5, "");
        const auto end = Clock::now() + std::chrono::milliseconds(1300);
        while (Clock::now() < end) {
            a.offer(frameOf(9), nowMs()); b.offer(frameOf(9), nowMs()); c.offer(frameOf(9), nowMs());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(!a.finished() && !b.finished() && !c.finished(), "luong model khong tu dung khi loi");
    }
    const auto has = [](Collector& col, const std::string& needle) {
        for (const auto& p : col.panels) if (p.notice.find(needle) != std::string::npos && !p.mask.empty()) return true;
        return false;
    };
    check(has(failing, "loi suy luan gia"), "suy luan nem -> thong bao");
    check(has(throwing, "onnx hong gia"), "nap model nem -> thong bao");
    check(has(empty, "--model"), "khong co model -> thong bao");
    check(throwing.size() >= 2 && throwing.size() <= 3, "thong bao 1 lan/giay");
}

void rateAndStop() {
    Collector out;
    std::unique_ptr<model_view::ModelWorker> worker(new model_view::ModelWorker(
        [] { return std::unique_ptr<model_view::IMaskModel>(new FrameModel(0, false)); }, std::ref(out), 5, ""));
    const auto end = Clock::now() + std::chrono::milliseconds(2000);
    while (Clock::now() < end) { worker->offer(frameOf(1), nowMs()); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    const auto count = out.size();
    check(count >= 8 && count <= 12, "infer-hz 5: " + std::to_string(count) + " ket qua/2 s");
    worker.reset(new model_view::ModelWorker(
        [] { return std::unique_ptr<model_view::IMaskModel>(new FrameModel(400, false)); }, std::ref(out), 5, ""));
    worker->offer(frameOf(1), nowMs());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto start = Clock::now();
    worker->requestStop();
    while (!worker->finished() && Clock::now() - start < std::chrono::seconds(2))
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    check(worker->finished() && ms < 600, "dung sau luot suy luan dang chay: " + std::to_string(ms) + " ms");
}
} // namespace

int main(int argc, char** argv) {
    try {
        const std::string part = argc == 2 ? argv[1] : "all";
        if (part == "all") { frameMatchAndNonBlocking(); errorsBecomeNotices(); rateAndStop(); }
        if (part == "all" || part == "latest") latestFrameCase();
        if (argc > 2 || (part != "all" && part != "latest")) return 2;
    }
    catch (const std::exception& e) { std::cerr << "HONG: ngoai le " << e.what() << '\n'; return 1; }
    if (failures) return 1;
    std::cout << "OK model_view worker\n";
    return 0;
}
