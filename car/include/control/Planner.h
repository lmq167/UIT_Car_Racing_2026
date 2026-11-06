#pragma once
#include "control/Detections.h"
#include "control/LaneController.h"
#include <deque>

namespace control {
constexpr int kStaleImageMs = 500;
constexpr const char* kMissingImageReason = "MAT_ANH";
enum class PlannerState { CHO_SAN_SANG, BAM_LAN, DUNG, CHO_DEN_XUAT_PHAT, DUNG_DEN_DO, DUNG_VAT_CAN, RE_TRAI, RE_PHAI };
const char *StateName(PlannerState state);
struct PlannerInput {
    cv::Mat mask;
    int64_t mask_ms=0;
    int64_t image_ms=-1;
    Detections detections;
    bool yolo_enabled=false; // Bên gọi P1 không có nguồn YOLO; auto/replay đặt tường minh.
    double yaw=0, encoder=0;
    double yaw_deg=0, odo_cm=0;
    bool sensor_ok=false;
    int64_t sensor_ms=-1;
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
    struct Vote { int64_t ms; bool red, green, left=false, right=false, barrier_near=false, barrier_seen=false; };
    std::deque<Vote> votes_;
    std::deque<bool> obstacle_votes_;
    int64_t last_mask_ms_=-1;
    int obstacle_clear_run_=0;
    enum class Swerve { NONE, LEFT, PASS };
    Swerve swerve_=Swerve::NONE;
    double pass_cm_=0, sent_cms_=0;
    int64_t swerve_ms_=-1;
    uint64_t last_frame_=0;
    int green_run_=0;
    int64_t red_ms_=-1, wait_since_ms_=-1;
    double red_x_=0;
    bool red_latched_=false, start_cleared_=false;
    bool barrier_done_=false; // Đã rẽ vì rào: bỏ qua rào tới khi hết phiếu rào (qua cả Space/G).
    bool approach_=false; // Đã xác nhận tiếp cận đỏ: giữ kẹp tới khi đỏ hết tươi/đủ xanh/dừng.
    LaneController lane_;
    PlannerState state_=PlannerState::CHO_SAN_SANG;
    std::string reason_="CHO_G";
    int turn_direction_=0;
    bool turning_=false, turn_fault_=false; // Lỗi rẽ giữ hết phiên, kể cả Space/G/R và Reset.
    double arm_odo_=0, turn_odo_=0, yaw0_=0, progress_odo_=0, sent_angle_=0;
    int64_t turn_ms_=-1, progress_ms_=-1, command_ms_=-1;
    void ClearTurn();
};
}
