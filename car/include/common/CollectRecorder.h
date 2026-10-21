#pragma once
#include "common/Config.h"
#include "common/Types.h"
#include <iosfwd>
#include <string>
#include <vector>

namespace common {
// Chỉ luồng giao diện gọi lớp này; xử lý/ghi ảnh không giữ khóa dữ liệu hay thiết bị.
class CollectRecorder {
public:
    CollectRecorder(const Config &c, const std::string &directory, std::istream &input, std::ostream &output);
    bool Key(int key, ControlCommand &command, Clock::time_point now=Clock::now());
    void Capture(const Snapshot &s, Clock::time_point now=Clock::now());
    cv::Mat View(const Snapshot &s) const;
    bool Recording() const { return recording_; }
    int Count() const { return count_; }
    int Pass() const { return pass_; }
    const std::string &Session() const { return session_; }
private:
    void RewriteLog() const;
    void Undo();
    Config c_;
    std::string directory_, session_;
    std::ostream &output_;
    struct Row { int pass; std::string frame, csv; };
    std::vector<Row> rows_;
    Clock::time_point started_=Clock::now(), last_capture_{}, undo_at_{}, quit_at_{};
    std::uint64_t last_frame_=0;
    int pass_=0, last_pass_=0, luot_=1, target_=1, count_=0, fps_=3;
    bool recording_=false, undo_armed_=false, quit_armed_=false;
    cv::Mat previous_;
};
}
