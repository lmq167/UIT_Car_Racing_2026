#pragma once
#include "control/Detections.h"
#include "control/LaneController.h"
#include <deque>

namespace control {
constexpr int kStaleImageMs = 500;
constexpr const char* kMissingImageReason = "MAT_ANH";
enum class PlannerState { CHO_SAN_SANG, BAM_LAN, DUNG, CHO_DEN_XUAT_PHAT, DUNG_DEN_DO, DUNG_VAT_CAN, RE_TRAI, RE_PHAI, GIU_HUONG };
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
    bool encoder_ok=false; // Encoder riêng: hữu hạn và tươi, không phụ thuộc yaw.
    int64_t sensor_ms=-1;
    bool go=false, stop=false;
};
struct Plan {
    PlannerState state=PlannerState::CHO_SAN_SANG;
    std::string reason="CHO_G";
    TrajMode mode=TrajMode::LANE;  // chế độ quỹ đạo đã gửi cho LaneController
    Command cmd;
    ControlDebug debug;
};
class Planner {
public:
    Plan Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params);
    void Reset(bool preserve_lights=false);
private:
    struct Vote { int64_t ms; bool red, green, left=false, right=false, barrier_near=false, barrier_seen=false, round=false, person=false, barrier_wide=false, round_big=false; };
    std::deque<Vote> votes_;
    std::deque<bool> obstacle_votes_;
    std::deque<bool> junction_votes_;
    int64_t last_junction_mask_ms_=-1;
    int64_t left_real_ms_=-1, right_real_ms_=-1;
    bool rb_turn_=false;
    int exit_run_=0;  // số mặt nạ liên tiếp thấy hành lang đường ra khi đang rẽ IMU
    int64_t last_exit_mask_ms_=-1;  // ý định rẽ đến từ lượt chờ sau vòng xuyến (giữ tới ngã T, không hủy theo quãng)  // lần gần nhất cạnh trái/phải ở hàng gần là cạnh thật
    int64_t last_mask_ms_=-1;
    int obstacle_clear_run_=0;
    enum class Swerve { NONE, LEFT, PASS };
    Swerve swerve_=Swerve::NONE;
    double pass_cm_=0, sent_cms_=0, return_cm_=-1;  // return_cm_ < 0: không ở pha VE_PHAI
    int64_t swerve_ms_=-1;
    double swerve_odo_=0;
    double swerve_progress_odo_=0; // mốc kiểm encoder tiến đủ 1 cm trong pha né/về
    int64_t swerve_progress_ms_=-1;
    bool swerve_odo_valid_=false; // mốc encoder của bước trước phải hợp lệ để lấy hiệu quãng
    uint64_t last_frame_=0;
    int green_run_=0;
    int64_t red_ms_=-1, wait_since_ms_=-1;
    double red_x_=0;
    bool red_latched_=false, start_cleared_=false;
    // Vòng xuyến: lần qua = từ lúc đủ phiếu round_barrier tới khi mất phiếu liên tục rb_lost_cm.
    // Bộ đếm chỉ về 0 khi khởi động chương trình (Quân đặt lại xe ở điểm xuất phát).
    bool rb_in_pass_=false, rb_losing_=false, rb_hold_spent_=false;
    int rb_passes_=0, rb_pending_=0;
    double rb_lost_odo_=0, rb_after_odo_=0, rb_yaw0_=0, rb_hold_odo_=0, rb_prog_odo_=0;
    int64_t rb_hold_ms_=-1, rb_prog_ms_=-1;
    bool rb_holding_=false; // Đang GIU_HUONG (bước trước): không bắt đầu rẽ/rào/né.
    bool person_stop_=false; // Đủ phiếu người đi bộ: lệnh tốc độ 0, giữ nguyên state.
    bool barrier_turn_=false; // Lượt rẽ hiện tại do rào khởi động.
    bool barrier_wide_turn_=false; // Rào lệch phải (work 2: vào đường 2 làn); sai: work 1 vào đường 1 làn.
    double return_e_=1; // Sai số làn lần cuối khi về làn phải (VE_PHAI).
    bool barrier_done_=false; // Đã rẽ vì rào: bỏ qua rào tới khi hết phiếu rào (qua cả Space/G).
    bool approach_=false; // Đã xác nhận tiếp cận đỏ: giữ kẹp tới khi đỏ hết tươi/đủ xanh/dừng.
    LaneController lane_;
    PlannerState state_=PlannerState::CHO_SAN_SANG;
    std::string reason_="CHO_G";
    int turn_direction_=0;
    bool turning_=false, turn_fault_=false; // Lỗi rẽ tạm: hết khi cảm biến tươi liên tục 1 s.
    bool corridor_turn_=false;
    int corridor_clear_=0;
    int64_t sensor_good_ms_=-1, sensor_bad_since_=-1;
    bool sensor_hold_=false; // Mẫu cảm biến hỏng thoáng qua: dừng giữ góc, chưa hủy rẽ.
    double arm_odo_=0, turn_odo_=0, yaw0_=0, progress_odo_=0, sent_angle_=0;
    int64_t turn_ms_=-1, progress_ms_=-1, command_ms_=-1;
    void ClearTurn();
};
}
