#pragma once
#include "common/Config.h"
#include "common/Types.h"
#include <atomic>
#include <utility>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
namespace common {
class RunLogger {
public:
    RunLogger(const Config &c, const std::string &source, const std::string &mode);
    ~RunLogger();
    void Record(const Snapshot &s);
    void Close();
    bool Failed() const { return failed_.load(); }
    const std::string &Directory() const { return directory_; }
private:
    void ImageThread();
    Config c_;
    std::string directory_;
    std::ofstream csv_;
    Clock::time_point started_=Clock::now();
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::pair<std::uint64_t,cv::Mat>> queue_;
    std::thread writer_;
    bool done_=false;
    std::atomic<bool> failed_{false};
    std::uint64_t last_saved_=0;
    bool collect_=false;
};
}
