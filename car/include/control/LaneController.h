#pragma once
#include "control/ControlParams.h"
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>

namespace control {
enum class LaneRef { RIGHT, LEFT };
struct Command { double cms=0, deg=0; };
struct RowDebug {
    int y=0;
    std::vector<double> lines;
    double left=-1, right=-1, center=-1;
    bool valid=false;
};
struct ControlDebug {
    std::vector<RowDebug> rows;
    double e=0, p=0, d=0, angle_deg=0, speed_cms=0;
    int valid_rows=0;
    std::string state="MAT_LAN", reason="CHUA_CO_LAN";
};
struct ControlResult { Command cmd; ControlDebug debug; };
class LaneController {
public:
    ControlResult Step(const cv::Mat &mask, int64_t mask_ms, int64_t now_ms,
                       const ControlParams &params, LaneRef ref=LaneRef::RIGHT);
    void Reset();
private:
    int64_t last_now_ms_=-1, last_mask_ms_=-1, last_valid_ms_=-1, regain_mask_ms_=-1;
    int64_t right_clip_since_ms_=-1;
    double angle_=0, error_=0;
    double filtered_derivative_=0;
    bool filter_ready_=false;
    double near_left_=0, near_right_=0;
    bool have_near_=false;
    bool lost_=false, derivative_ready_=false, right_lost_=false;
    int regain_=0;
    LaneRef reference_=LaneRef::RIGHT;
};
}
