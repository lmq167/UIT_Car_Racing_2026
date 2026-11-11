#pragma once
#include "control/Detections.h"
#include "control/LaneController.h"
#include "control/ParkScan.h"
#include <deque>

namespace control {
constexpr int kStaleImageMs = 500;
constexpr const char* kMissingImageReason = "MAT_ANH";
enum class PlannerState { CHO_SAN_SANG, BAM_LAN, DUNG, CHO_DEN_XUAT_PHAT, DUNG_DEN_DO, DUNG_VAT_CAN, RE_TRAI, RE_PHAI, GIU_HUONG,
                          DO_XE, DO_XE_XONG };  // DO_XE: chờ/lùi vào ô (chặng 8); DO_XE_XONG: đã đỗ, đứng yên
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
    TofSlot tof[4];  // 4 ToF theo khe BTC 0..3 ([0] trái-sau, [1] trái-trước, [2] phải-trước, [3] phải-sau); chỉ đọc khi xe đứng
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
    // Lộ trình chặng vòng xuyến (chỉ đọc, cho test/phát lại): chặng 0-7 và lý do đổi gần nhất.
    int RouteStep() const { return route_step_; }
    const std::string &RouteNote() const { return route_note_; }
    // Hướng rẽ đang giữ (0 không có, 1 phải, -1 trái; chỉ đọc, để ghi log chẩn đoán).
    int TurnDirection() const { return turn_direction_; }
    // Ghi chú quét ô ToF gần nhất (để ghi log); rỗng nếu chưa quét.
    const std::string &ParkNote() const { return park_note_; }
private:
    // Lộ trình theo hướng lượng tử 90° (Bắc = yaw lúc mở lần qua đảo đầu tiên). Không bị Reset/Space xóa.
    int route_step_=0, route_q_=0, route_cand_q_=0;  // route_q_: hướng (bội 90°) của chặng hiện tại
    int64_t route_cand_ms_=-1;  // mốc bắt đầu hướng ứng viên ổn định; <0: chưa ổn định
    double route_cand_odo_=0;   // odo lúc bắt đầu hướng ứng viên (chỉ đếm khi xe thực sự lăn bánh)
    double route_yaw_ref_=0;
    bool route_anchored_=false, route_g_seen_=false, route_sync_=false, route_ambig_=false;
    bool route_check_rb_=false, route_pass_seen_=false;
    std::string route_note_;
    void RouteStable(int q);
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
    int ahead_clear_lots_=-1;  // -1: không dừng vì vật trước mặt; >=0: số lô YOLO tươi liên tiếp vắng vật gần
    enum class Swerve { NONE, LEFT, PASS };
    Swerve swerve_=Swerve::NONE;
    double pass_cm_=0, sent_cms_=0, return_cm_=-1;  // return_cm_ < 0: không ở pha VE_PHAI
    bool swerve_right_seen_=false;  // đã thấy vật ở nửa phải ảnh trong pha trái (Lỗi 3, 11/11)
    double swerve_peak_w_=0;  // bề rộng hộp vật cản lớn nhất (theo ảnh) từ lúc bắt đầu pha trái; vật nhỏ hơn nửa mức này là vật xa, không giữ pha trái
    int64_t swerve_ms_=-1;
    double swerve_odo_=0;
    double swerve_progress_odo_=0; // mốc kiểm encoder tiến đủ 1 cm trong pha né/về
    int64_t swerve_progress_ms_=-1;
    bool swerve_odo_valid_=false; // mốc encoder của bước trước phải hợp lệ để lấy hiệu quãng
    uint64_t last_frame_=0, last_obstacle_frame_=0; // Lô YOLO đã xét cho vật cản.
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
    bool person_latched_=false; // Giữ dừng tới khi lô tươi xác nhận vắng người đủ 2 s.
    int64_t person_seen_ms_=-1;
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
    std::deque<std::pair<double,double>> pre_turn_;  // (odo, yaw) khi bám làn chờ rẽ, trong kPreTurnWindowCm gần nhất
    double turn_steer_scale_=1;  // hệ số góc rẽ cố định theo phần yaw đã quay sẵn trước RE_* (lỗi 2)
    // Cua L phải sau cầu trước VX1 (Quân duyệt 11/11): đảo thấy ở mép phải rồi các hàng gần phủ kín ảnh.
    bool l_armed_=false, l_done_=false, l_turning_=false;
    double l_seen_odo_=0;
    int l_right_run_=0, l_open_run_=0;
    // Đỗ xe lùi song song (chặng 8, Quân duyệt viết với số tạm 11/11).
    enum class ParkPhase { NONE, TIEN, QUET, CHO, LUI1, LUI2, XONG };
    ParkPhase park_phase_=ParkPhase::NONE;
    double park_odo0_=0, park_yaw0_=0, park_prog_odo_=0;
    int64_t park_ms0_=-1, park_prog_ms_=-1;
    int park_run_=0;  // số lô YOLO tươi liên tiếp có biển parking
    // Quét ô bằng ToF khi xe đứng, tiến từng bước (Quân duyệt 11/11): mode 0 = tìm ô trống (bước 50 cm), 1 = bò 10 cm tìm mép xa, 2 = đi quá mép.
    int park_mode_=0, park_unknown_=0;
    double park_step_cm_=0, park_scan_cm_=0, park_creep_cm_=0, park_cap_cms_=0;
    int64_t park_scan_ms0_=-1;
    std::string park_note_;
    void ClearTurn();
};
}
