#pragma once
#include "model_worker.h"
#include "control/LaneController.h"
#include <condition_variable>
#include <deque>
#include <thread>

namespace auto_drive {
void warningNonBlocking(const char* text) noexcept;
std::string sha256(const std::string& path);
struct Statistics {
    double camera_fps=0, road_ms=0, yolo_ms=0, loop_p95_ms=0, rss_mb=0, free_mb=0;
};
class LogWriter {
    std::string root_, yaml_, info_, engine_, yoloInfo_, yoloEngine_;
    std::string unetOnnxSha256_, yoloOnnxSha256_;
    bool yoloEnabled_;
    std::function<void(const char*,const std::string&)> trace_;
    std::mutex mutex_;
    std::condition_variable available_;
    std::deque<ModelResult> images_;
    std::deque<std::string> csv_;
    std::deque<control::Detections> detections_;
    std::atomic<bool> quit_{false};
    Mailbox<Statistics> statistics_;
    std::atomic<int> recordEvery_{5};
    std::atomic<bool> recordP95_{false};
    std::thread worker_;
    void run() noexcept;
public:
    std::string folder;
    std::atomic<bool> ready{false}, finished{false}, imagesEnabled{true};
    std::atomic<uint64_t> dropped{0}, csvDropped{0}, detectionsDropped{0}, jpegFail{0}, csvFail{0};
    std::atomic<double> freeMb{0}, rssMb{0};
    LogWriter(const Options&, const std::string& yaml,
              std::function<void(const char*,const std::string&)> trace);
    bool tryImage(const ModelResult& image) noexcept;
    void offerDetections(const control::Detections&) noexcept;
    void offerControl(const std::string& row) noexcept;
    void offerStatistics(const Statistics&, int every, bool p95=false);
    void requestStop() noexcept;
    ~LogWriter();
};
std::string scanJson(const control::ControlDebug&, int width, int height);
}
