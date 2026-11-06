#pragma once
#include "control/Detections.h"
#include "control/LaneController.h"
#include <deque>

namespace control {
constexpr int kStaleImageMs = 500;
constexpr const char* kMissingImageReason = "MAT_ANH";
enum class PlannerState { CHO_SAN_SANG, BAM_LAN, DUNG, CHO_DEN_XUAT_PHAT, DUNG_DEN_DO, DUNG_VAT_CAN };
const char *StateName(PlannerState state);
struct PlannerInput {
    cv::Mat mask;
    int64_t mask_ms=0;
    int64_t image_ms=-1;
    Detections detections;
    bool yolo_enabled=false; // Bên gọi P1 không có nguồn YOLO; auto/replay đặt tường minh.
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
    void Reset(bool preserve_lights=false);
private:
    struct Vote { int64_t ms; bool red, green, barrier_near; };
    std::deque<Vote> votes_;
    std::deque<bool> obstacle_votes_;
    int64_t last_mask_ms_=-1;
    int obstacle_clear_run_=0;
    enum class Swerve { NONE, LEFT, PASS };
    Swerve swerve_=Swerve::NONE;
    bool barrier_turn_=false, barrier_done_=false;
    double pass_cm_=0, sent_cms_=0;
    int64_t swerve_ms_=-1;
    uint64_t last_frame_=0;
    int green_run_=0;
    int64_t red_ms_=-1, wait_since_ms_=-1;
    double red_x_=0;
    bool red_latched_=false, start_cleared_=false;
    LaneController lane_;
    PlannerState state_=PlannerState::CHO_SAN_SANG;
    std::string reason_="CHO_G";
};
}
