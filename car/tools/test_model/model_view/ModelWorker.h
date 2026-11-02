#pragma once
#include "IMaskModel.h"
#include <opencv2/core.hpp>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace model_view {
struct ModelPanels {
    cv::Mat mask, overlay;
    // Rỗng khi có kết quả; có lý do khi không có model hoặc suy luận lỗi.
    std::string notice;
    double inferMs = 0, resultAgeMs = 0;
    uint64_t results = 0;
};

// Luồng riêng: lấy khung MỚI NHẤT, bỏ khung cũ, chạy theo hz; mọi lỗi chỉ thành thông báo trên panel.
class ModelWorker {
public:
    using Factory = std::function<std::unique_ptr<IMaskModel>()>;
    using Sink = std::function<void(const ModelPanels&)>;
    ModelWorker(Factory factory, Sink sink, int hz, std::string label);
    ~ModelWorker();
    // Gọi từ vòng điều khiển: không bao giờ chờ; bên kia đang giữ hộp thư thì bỏ khung.
    void offer(const cv::Mat& bgr, int64_t capturedMs) noexcept;
    void requestStop() noexcept;
    // Luồng kẹt trong suy luận: chủ giữ đối tượng sống tới _exit, không hủy (hàm hủy join).
    bool finished() const noexcept { return finished_.load(); }
private:
    void run() noexcept;
    bool take(cv::Mat& bgr, int64_t& capturedMs);
    void publish(const ModelPanels& panels) noexcept;
    void notice(const std::string& reason) noexcept;
    void process(IMaskModel& model, const cv::Mat& bgr, int64_t capturedMs);
    Factory factory_;
    Sink sink_;
    int periodMs_;
    std::string label_;
    std::mutex mutex_;
    std::condition_variable changed_;
    cv::Mat mailbox_;
    int64_t mailboxMs_ = 0;
    bool quit_ = false;
    std::atomic<bool> finished_{false};
    uint64_t results_ = 0;
    int64_t firstResultMs_ = 0;
    std::thread worker_;
};
} // namespace model_view
