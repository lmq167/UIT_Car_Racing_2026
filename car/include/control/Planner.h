#pragma once
#include "control/Detections.h"
#include "control/LaneController.h"

namespace control {
enum class PlannerState { CHO_SAN_SANG, BAM_LAN, DUNG };
const char *StateName(PlannerState state);
struct PlannerInput {
    cv::Mat mask;
    int64_t mask_ms=0;
    int64_t image_ms=-1;
    Detections detections;
    double yaw=0, encoder=0;
    bool go=false, stop=false;
    LaneRef reference=LaneRef::RIGHT;
};
struct Plan {
    PlannerState state=PlannerState::CHO_SAN_SANG;
    std::string reason="CHO_G";
    LaneRef reference=LaneRef::RIGHT;
    Command cmd;
    ControlDebug debug;
};
class Planner {
public:
    Plan Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params);
    void Reset();
private:
    LaneController lane_;
    PlannerState state_=PlannerState::CHO_SAN_SANG;
    std::string reason_="CHO_G";
};
}
