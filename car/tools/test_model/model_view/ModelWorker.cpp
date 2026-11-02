#include "ModelWorker.h"
#include "MaskRender.h"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <csignal>
#include <exception>
#include <pthread.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace model_view {
namespace {
constexpr int kNoticePeriodMs = 1000;
using Clock = std::chrono::steady_clock;
int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
} // namespace

ModelWorker::ModelWorker(Factory factory, Sink sink, int hz, std::string label)
    : factory_(std::move(factory)), sink_(std::move(sink)), periodMs_(1000 / hz), label_(std::move(label)) {
    // Tín hiệu dừng xe phải tới luồng chính; luồng model không nhận hộ.
    sigset_t blocked, original;
    sigfillset(&blocked);
    if (pthread_sigmask(SIG_BLOCK, &blocked, &original) != 0) throw std::runtime_error("Khong chan duoc tin hieu");
    try { worker_ = std::thread([this] { run(); }); }
    catch (...) { pthread_sigmask(SIG_SETMASK, &original, nullptr); throw; }
    pthread_sigmask(SIG_SETMASK, &original, nullptr);
}

ModelWorker::~ModelWorker() {
    requestStop();
    if (worker_.joinable()) worker_.join();
}

void ModelWorker::offer(const cv::Mat& bgr, int64_t capturedMs) noexcept {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    mailbox_ = bgr; // Tham chiếu: khung camera bất biến, mỗi lượt đọc cấp bộ đệm mới.
    mailboxMs_ = capturedMs;
    lock.unlock();
    changed_.notify_one();
}

void ModelWorker::requestStop() noexcept {
    { std::lock_guard<std::mutex> lock(mutex_); quit_ = true; }
    changed_.notify_all();
}

// Trả false khi phải thoát. Không có khung mới thì bgr rỗng.
bool ModelWorker::take(cv::Mat& bgr, int64_t& capturedMs) {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, std::chrono::milliseconds(periodMs_), [this] { return quit_ || !mailbox_.empty(); });
    if (quit_) return false;
    bgr = std::move(mailbox_);
    mailbox_ = cv::Mat();
    capturedMs = mailboxMs_;
    return true;
}

void ModelWorker::publish(const ModelPanels& panels) noexcept {
    try { sink_(panels); } catch (...) {} // Cửa sổ phụ hỏng không được lan sang xe.
}

void ModelWorker::notice(const std::string& reason) noexcept {
    try {
        ModelPanels panels;
        panels.notice = reason;
        panels.mask = renderNotice(reason);
        panels.overlay = panels.mask;
        publish(panels);
    } catch (...) {}
}

void ModelWorker::process(IMaskModel& model, const cv::Mat& bgr, int64_t capturedMs) {
    const auto& info = model.info();
    // Thu nhỏ TRƯỚC khi suy luận và từ chính khung đó: panel không lệch khung dù model chậm.
    cv::Mat thumb;
    cv::resize(bgr, thumb, cv::Size(kThumbWidth, kThumbHeight), 0, 0, cv::INTER_AREA);
    const auto start = Clock::now();
    const cv::Mat mask = model.infer(bgr);
    const double inferMs = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    ModelPanels panels;
    panels.mask = renderMaskPanel(mask, info);
    panels.overlay = renderOverlayPanel(thumb, mask, info);
    panels.inferMs = inferMs;
    const int64_t now = nowMs();
    panels.resultAgeMs = double(now - capturedMs);
    panels.results = ++results_;
    if (results_ == 1) firstResultMs_ = now;
    const double fps = now > firstResultMs_ ? 1000.0 * double(results_ - 1) / double(now - firstResultMs_) : 0.0;
    drawLine(panels.mask, 0, "Model Mask  " + label_);
    drawLine(panels.overlay, 0, "Model Overlay  " + label_);
    std::ostringstream hw, timing;
    hw << info.name << " input " << info.inputH << "x" << info.inputW << (info.fakeData ? " FAKE_DATA" : "");
    timing.setf(std::ios::fixed); timing.precision(1);
    timing << "infer " << inferMs << " ms  " << fps << " fps  tuoi " << panels.resultAgeMs << " ms";
    drawLine(panels.overlay, 1, hw.str());
    drawLine(panels.overlay, 2, timing.str());
    publish(panels);
}

void ModelWorker::run() noexcept {
    std::unique_ptr<IMaskModel> model;
    std::string reason;
    try {
        model = factory_();
        if (!model) reason = "chua cap --model";
    } catch (const std::exception& e) { reason = e.what(); }
    catch (...) { reason = "loi khong ro khi nap model"; }

    int64_t nextNotice = 0, nextInfer = 0;
    try {
        cv::Mat bgr;
        int64_t capturedMs = 0;
        while (take(bgr, capturedMs)) {
            const int64_t now = nowMs();
            if (!reason.empty() && now >= nextNotice) {
                notice(reason);
                nextNotice = now + kNoticePeriodMs;
            }
            if (!model || bgr.empty() || now < nextInfer) continue;
            nextInfer = now + periodMs_;
            try {
                process(*model, bgr, capturedMs);
                reason.clear();
            } catch (const std::exception& e) { reason = e.what(); }
            catch (...) { reason = "loi khong ro khi suy luan"; }
        }
    } catch (...) {}
    finished_ = true;
}
} // namespace model_view
