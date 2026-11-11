#include "control/Planner.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {
constexpr double kBarrierConfMin=0.5; // Hằng tin cậy rào/vòng xuyến như vật YOLO khác.
constexpr double kObstacleConfMin=0.5;
constexpr double kSwervePresentFrac=0.5; // TẠM: hộp hẹp hơn nửa bề rộng lớn nhất từ lúc bắt đầu né không giữ pha trái.
// TẠM: tâm hộp vật làn phải. Log 10/11 (1794302164): vật làn phải khi xe lệch hướng có tâm 0,43-0,45 (ngưỡng 0,45 làm không né),
// vật làn trái có tâm <= 0,18; chọn 0,30 nằm giữa hai nhóm.
constexpr double kObstacleRightLaneCx=0.30;
constexpr double kPersonConfMin=0.5; // Hằng như rào; tên lớp người chờ model mới.
// Đáy hộp người kích dừng. Log 10/11: 0,98 dừng khi hình nộm cách ~14 cm; 0,85 dừng ~22 cm (sớm 0,5 s), thử xe dừng được;
// Quân chọn 0,83 cho an toàn hơn (10/11).
constexpr double kPersonStopBottom=0.83;
constexpr int64_t kPersonReleaseMs=2000; // Lô tươi xác nhận vắng người đủ 2 s mới nhả.
constexpr int64_t kSensorGlitchMs=250;  // Mẫu cảm biến hỏng ngắn hơn mức này thì dừng giữ góc, dài hơn thì hủy rẽ (Quân duyệt 07/11).
// Rào/vòng xuyến gần theo bề rộng hộp (ảnh 1280): đáy hộp rào xa vẫn 0,52-0,55 vì camera thấp (log 09/11 phiên 7085).
// Rào hẹp 115 px, rào lệch phải 90 px trùng thời điểm ngưỡng đáy hộp 0,60/0,65 đã chỉnh 07/11; đảo xa 100-233 px, vào thật >= 297 px.
constexpr double kNarrowBarrierMinW=115, kWideBarrierMinW=90, kRoundMinW=280;
// Giảm tốc trước rào ~2 s trước điểm rẽ (Quân 09/11): rào hẹp ~100 px (~87 cm), rào lệch phải ~80 px (log 09/11 sáng).
constexpr double kNarrowBarrierSlowW=100, kWideBarrierSlowW=80;
// Giao lộ trên mặt nạ (log 09/11 sáng, xe đi thẳng qua): đường phía trước kết thúc ở >= 0,69 chiều cao khi tới ngã T.
constexpr double kRoadEndFrac=0.69;
constexpr double kSideRoadEdgeFrac=0.10;  // đường ngang ở ngã T chạm vùng 10 % mép ảnh phía rẽ
constexpr int64_t kRealSideMs=2000;
// Lượt rẽ IMU (log 09/11 chiều tối): 4 cm/s x 8 s chỉ quay 18-32° rồi hết hạn → bám làn nhận khi mới quay một phần.
// Rẽ 8 cm/s (chỉ trong RE_*), giao bám làn khi đã quay >= 45° VÀ mặt nạ thấy hành lang ra (>= 3 hàng 2 cạnh thật, lệch <= 0,15).
constexpr double kTurnCms=8.0, kHandoverTolDeg=10.0, kExitOffsetFrac=0.15;
// Quân 11/11: IMU là phản hồi chính của cua. Mặt nạ chỉ giao lại khi yaw đã tới (đích - kHandoverTolDeg); trong cua, góc lái được
// tăng khi chậm tiến độ yaw so với quãng còn lại (mô hình xe: dyaw = d·tan(lái)/L_eff, L_eff 31 cm đo từ log m4), không vượt steer_max_deg.
constexpr double kTurnLeffCm=31.0, kTurnPaceMargin=0.85;
constexpr double kSwerveCms=8.0;  // Quân 10/11: tốc độ pha né (NE_TRAI/NE_QUA/VE_PHAI), trước đó dùng approach_cms 5; vẫn bị chặn bởi max_speed_cms và tốc độ bám làn.
constexpr int kExitRealRows=3;  // cạnh phía rẽ phải là cạnh thật trong 2 s gần nhất thì chỗ mở mới là giao lộ
constexpr double kSignMinWidthFrac=45.0/1280;  // TẠM: biển hẹp hơn (theo bề rộng ảnh) là biển xa/chặng khác, không tính phiếu; biển giả đo được rộng <= 36 px.
// Góc lái rẽ theo hình học sa bàn (làn 30 cm, bo R=50): rẽ phải/vào 1 làn cần bán kính đường xe ~65 cm (lái nhiều hơn ~12%),
// rẽ trái vào đường 2 làn (rào) ~95 cm (lái ít hơn ~20%); biển/vòng xuyến rẽ trái chưa biết số làn nên giữ turn_steer_deg.
// Rào work 1 (vào đường 1 làn) lệch giữa (tâm hộp 0,43-0,54), work 2 (vào đường 2 làn) lệch phải (0,70-0,71): đo ở hai phiên 07/11.
constexpr double kWideBarrierCx=0.55, kWideBarrierMaxCx=0.82, kNarrowBarrierEarly=0.05;
constexpr double kBarrierBlockMinCx=0.25, kBarrierBlockMaxCx=0.75;  // dải tâm hộp rào còn chắn trước (chặn tiếp quản)
// Rẽ phải thoát sớm hơn một chút: log 4 lần rẽ phải xong ở -80 độ rồi bộ bám làn phải trả lại 5-8 độ.
constexpr double kRightTurnYawFactor=0.92;
// Lộ trình chặng vòng xuyến (Quân 10/11, T94): hướng từng chặng theo bội 90°, Bắc = 0, trái dương (yaw tăng khi rẽ trái).
// c0 trước VX1, c1 Đông, c2 Bắc (T trái), c3 Tây (T trái), c4 Nam (VX2), c5 Tây, c6 Bắc, c7 Đông = hết ý định.
constexpr int kRouteSteps=8;
constexpr int kRouteHeading[kRouteSteps]={0,-1,0,1,2,1,0,-1};
constexpr double kRbHoldAlignDeg=40;   // TẠM, CHƯA ĐO: lần qua mở sau G-sau-Space vẫn giữ hướng nếu yaw lệch trục lộ trình không quá từng này
constexpr double kRouteAxisDeg=30;     // TẠM, CHƯA ĐO: cách trục <= 30° mới coi là đang theo trục đó
constexpr int64_t kRouteStableMs=500;  // TẠM, CHƯA ĐO: giữ cùng trục liên tục từng này mới tính hướng ổn định
constexpr double kRouteMoveCm=2;       // TẠM, CHƯA ĐO: trong lúc ổn định bánh phải lăn ít nhất từng này (không đếm khi đứng/nhấc)
int Mod4(int q) { return ((q%4)+4)%4; }
constexpr double kReturnAlignedError=0.12, kReturnCapFactor=5;
// Lỗi 1 (log lan3 10/11): vật thứ hai trước mặt trên làn trái trong NE_QUA. TẠM, CHƯA ĐO trên sa bàn:
// tâm hộp trong [0,30; 0,70] và đáy hộp >= 0,60 chiều cao ảnh thì kết thúc PASS khi đã đi >= 0,85·obstacle_pass_cm
// (0,85·85 ≈ 72 cm sau khi bắt đầu PASS; PASS có thể bắt đầu sớm 15–17 cm so với lúc vật rời ảnh nên hở đuôi xe CHƯA ĐO, cần đo sa bàn).
constexpr double kPassAheadMinCx=0.30, kPassAheadMaxCx=0.70, kPassAheadY2=0.60, kPassMinFrac=0.85;
// Chưa đủ quãng tối thiểu mà vật trước mặt đã rất gần (đáy >= 0,85): dừng KHONG_DUONG_NE (an toàn, như MEDIUM-1 P1).
constexpr double kPassStopY2=0.85;
// Lỗi 3 (log 11/11 m3, 2 phiên): vật 1 ở làn phải đã qua nhưng vật 2 (trước mặt, làn trái) vẫn giữ NE_TRAI tới sát vật 2.
// Khi đã thấy vật ở nửa phải (tâm >= 0,60), pha trái chỉ giữ bởi hộp ở nửa phải; hộp trước mặt/bên trái không giữ (Quân 11/11).
// TẠM, CHƯA ĐO trên sa bàn (cùng obstacle_near_w_frac): ngưỡng tâm 0,60; giữ pha trái khi hộp phải >= 0,5·near_w (kSwervePresentFrac),
// quay lại pha trái từ PASS khi hộp phải >= 1,0·near_w (trễ có chủ ý để tránh dao động); vật trước mặt sát (ahead_stop) luôn ưu tiên dừng.
constexpr double kHoldRightCx=0.60;
// Lỗi 2 (log 10/11 lan2/lan3, 6 lượt rẽ phải): bám làn đã quay sẵn 9-20° theo miệng ngã trong 60 cm trước RE_*,
// rồi cung lái cố định cắt góc. Quân chọn C (11/11): giảm góc rẽ cố định theo phần đã quay sẵn. TẠM, CHƯA ĐO trên sa bàn:
// hệ số = 1 - quay_sẵn/90°, kẹp [0,75; 1] (quay sẵn 20° -> 0,78; 25° x 1,12 = 28° -> 21,8°).
constexpr double kPreTurnWindowCm=60, kPreTurnRefDeg=90, kPreTurnMinScale=0.75;
// Cua L phải hai làn sau cầu, ngay trước VX1 (Quân duyệt 11/11; log 10/11 10102 + t94 12471, ảnh p0088). TẠM, CHƯA ĐO:
// - Đảo (round_barrier, cỡ bất kỳ) có tâm hộp >= 0,80 ở vote_min lô tươi liên tiếp trước khi neo lộ trình -> chờ cua L.
//   Log: đảo nhỏ (rộng 0,10-0,17, tâm 0,85-0,95) thấy ~10 s rồi khuất 59-76 cm trước cua.
// - Trong 150 cm từ lần cuối thấy đảo bên phải, >= 3 trong các hàng gần có hai biên ở mép ảnh (góc cua rộng phủ kín ảnh,
//   bám làn mất thông tin và kéo sang làn trái) ở vote_min mặt nạ liên tiếp -> rẽ phải cung cố định (như RE_PHAI), một lần mỗi G.
constexpr double kLBarrierCx=0.80, kLArmCm=150;
constexpr int kLOpenRows=3;
// Đỗ xe lùi song song (park_design_1011.md; Quân duyệt viết với số tạm 11/11). TẠM, CHƯA ĐO trên xe:
// biển parking rộng >= 0,05 ảnh ở vote_min lô tươi -> bám làn park_forward_cm -> dừng 600 ms -> lùi cung 1 lái về phía ô
// tới khi quay park_arc_deg -> lùi cung 2 lái ngược tới khi về hướng ban đầu ± 3° -> đứng yên (DO_XE_XONG).
// Mỗi cung lùi: tối đa 80 cm, 15 s; không nhích >= 1 cm trong 2 s thì dừng; cảm biến lỗi thì dừng.
constexpr double kParkSignMinW=0.05, kParkYawDoneDeg=3, kParkPhaseMaxCm=80;
constexpr int64_t kParkWaitMs=600, kParkPhaseMaxMs=15000, kParkStallMs=2000;
// Quét ô ToF khi xe đứng (Quân duyệt 11/11). TẠM, CHƯA ĐO: bước, ngưỡng vật, thời gian chờ mẫu.
constexpr double kParkStepCm=50, kParkCreepCm=10, kParkPastCm=10, kParkOccMm=400, kParkMaxScanCm=400, kParkMaxCreepCm=80;
constexpr double kParkStepCms=8, kParkCreepCms=5;
constexpr int64_t kParkSettleMs=500, kParkScanMaxMs=3000;
constexpr int kParkTofAgeMs=300;
double TurnSteer(const ControlParams &p, int direction, bool barrier_turn, bool wide_barrier) {
    const double s=direction<0 ? p.turn_steer_deg*1.12 : barrier_turn ? p.turn_steer_deg*(wide_barrier ? 0.8 : 1.12) : p.turn_steer_deg;
    return std::min(s,p.steer_max_deg);
}
// Sai số góc gói về [-180, 180).
double WrapDeg(double d) { d=std::fmod(d+180.0,360.0); if(d<0) d+=360.0; return d-180.0; }

}

const char *StateName(PlannerState state) {
    switch(state) {
        case PlannerState::CHO_SAN_SANG: return "CHO_SAN_SANG";
        case PlannerState::BAM_LAN: return "BAM_LAN";
        case PlannerState::DUNG: return "DUNG";
        case PlannerState::CHO_DEN_XUAT_PHAT: return "CHO_DEN_XUAT_PHAT";
        case PlannerState::DUNG_DEN_DO: return "DUNG_DEN_DO";
        case PlannerState::DUNG_VAT_CAN: return "DUNG_VAT_CAN";
        case PlannerState::RE_TRAI: return "RE_TRAI";
        case PlannerState::RE_PHAI: return "RE_PHAI";
        case PlannerState::GIU_HUONG: return "GIU_HUONG";
        case PlannerState::DO_XE: return "DO_XE";
        case PlannerState::DO_XE_XONG: return "DO_XE_XONG";
    }
    return "DUNG";
}
// Hướng q (bội 90° so với Bắc) vừa ổn định: đồng bộ sau G, giải mơ hồ, hoặc tiến một chặng.
void Planner::RouteStable(int q) {
    const int s=route_step_, d=q-route_q_;
    auto match=[&](int i) { return i>=0 && i<kRouteSteps && Mod4(kRouteHeading[i])==Mod4(q); };
    if(route_sync_) {  // G sau Space (R1: chờ tới khi hướng ổn định mới xét)
        route_sync_=route_ambig_=route_pass_seen_=false; route_q_=q;
        if(match(s)) route_note_="SYNC_R2";
        else if(match(s-1)!=match(s+1)) { route_step_=match(s-1) ? s-1 : s+1; route_note_="SYNC_R3"; }
        else if(Mod4(q)==2) { route_step_=4; route_note_="SYNC_R4"; }
        else { route_ambig_=true; route_note_="SYNC_AMBIG"; }
        return;  // R5 (đảo phía trước) còn xét tới chặng kế tiếp
    }
    if(route_ambig_) {  // góc kế tiếp xác định chặng; Bắc→Đông có ở c0 và c6, phân biệt bằng lần qua đảo
        int found=-1, count=0;
        for(int i=0;i+1<kRouteSteps;++i)
            if(Mod4(kRouteHeading[i])==Mod4(route_q_) && kRouteHeading[i+1]-kRouteHeading[i]==d) { if(found<0) found=i; ++count; }
        route_q_=q;
        const int next=count ? (count>1 ? (route_pass_seen_ ? 0 : 6) : found)+1 : -1;
        // Chặng có ý định rẽ (c2, c3) cần bằng chứng mạnh hơn một góc sau mơ hồ: vẫn mơ hồ, chỉ bám làn.
        if(next<0 || next==2 || next==3) { route_note_="SYNC_AMBIG"; return; }
        route_step_=next; route_ambig_=false; route_note_="SYNC_GOC";
    } else if(s+1<kRouteSteps && d==kRouteHeading[s+1]-kRouteHeading[s]) {
        ++route_step_; route_q_=q; route_note_="BUOC";
    } else {  // sai dấu/nhảy 2 nấc/đã hết: không tiến; thiếu bằng chứng vị trí nên chuyển mơ hồ (chỉ bám làn)
        route_note_="HUONG_SAI"; route_ambig_=true; route_q_=q; return;
    }
    route_check_rb_=route_pass_seen_=false;
}
void Planner::ClearTurn() {
    junction_votes_.clear(); last_junction_mask_ms_=-1; exit_run_=0; last_exit_mask_ms_=-1;
    corridor_turn_=false; corridor_clear_=0;
    turn_direction_=0; turning_=false; barrier_turn_=false; barrier_wide_turn_=false; rb_turn_=false;
    arm_odo_=turn_odo_=yaw0_=progress_odo_=0; turn_ms_=progress_ms_=-1;
    pre_turn_.clear(); turn_steer_scale_=1; l_turning_=false;
    for(auto &vote:votes_) vote.left=vote.right=false;
}
void Planner::Reset(bool preserve_lights) {
    ClearTurn(); sent_angle_=0; command_ms_=-1; sensor_bad_since_=-1; sensor_hold_=false; left_real_ms_=right_real_ms_=-1;
    lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; green_run_=0; wait_since_ms_=-1;
    obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0; ahead_clear_lots_=-1;
    swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; swerve_odo_valid_=false; approach_=false; person_stop_=false; person_latched_=false; person_seen_ms_=-1; return_cm_=-1;
    l_armed_=l_done_=false; l_seen_odo_=0; l_right_run_=l_open_run_=0;
    park_phase_=ParkPhase::NONE; park_run_=0; park_ms0_=park_prog_ms_=-1;
    swerve_peak_w_=0; swerve_right_seen_=false;
    park_mode_=0; park_unknown_=0; park_step_cm_=park_scan_cm_=park_creep_cm_=park_cap_cms_=0; park_scan_ms0_=-1; park_note_.clear();
    rb_hold_spent_=true; rb_holding_=false;  // sau Space/G/R không giữ hướng bằng yaw0 cũ
    sensor_good_ms_=-1;
    // Epoch đổi phải xoá lịch sử làn, nhưng Space/G/R không xoá pha đèn P2.
    if(preserve_lights) return;
    votes_.clear(); red_ms_=-1; red_x_=0; last_frame_=last_obstacle_frame_=0; red_latched_=false; start_cleared_=false; barrier_done_=false;
}
Plan Planner::Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params) {
    params.Validate();
    Plan plan;
    const bool image_fresh=input.image_ms>=0 && input.image_ms<=now_ms && now_ms-input.image_ms<=kStaleImageMs;
    const bool mask_fresh=!input.mask.empty() && input.mask_ms>=0 && input.mask_ms<=now_ms &&
                          now_ms-input.mask_ms<=params.mask_max_age_ms;
    const bool sensor_bad=!input.sensor_ok || !std::isfinite(input.yaw_deg) || !std::isfinite(input.odo_cm) ||
        input.sensor_ms<0 || input.sensor_ms>now_ms || now_ms-input.sensor_ms>200;
    bool entered_wait=false, released=false, barrier_slow=false, barrier_fault_stop=false, rb_entered=false;
    if(input.stop) {
        ClearTurn(); sent_angle_=0; command_ms_=-1; rb_hold_spent_=true; rb_holding_=false;
        rb_in_pass_=rb_losing_=false;  // Space: bỏ lần qua vòng xuyến đang dở, không tính thành lần qua sau khi đặt lại xe (log 09/11 138 s)
        for(auto &vote:votes_) vote.round=vote.round_big=false;  // phiếu đảo cũ không mở lại lần qua sau G
        state_=PlannerState::DUNG; reason_="SPACE"; lane_.Reset(); green_run_=0; park_phase_=ParkPhase::NONE; park_run_=0;
    } else if(state_!=PlannerState::CHO_SAN_SANG && state_!=PlannerState::DUNG && !image_fresh) {
        ClearTurn();
        state_=PlannerState::DUNG; reason_=kMissingImageReason; lane_.Reset(); green_run_=0; park_phase_=ParkPhase::NONE;
    } else if(input.go && (state_==PlannerState::CHO_SAN_SANG || state_==PlannerState::DUNG)) {
        if(!image_fresh || !mask_fresh) {
            if(params.turn_enable && sensor_bad) { ClearTurn(); turn_fault_=true; rb_pending_=0; sensor_good_ms_=-1; }
            plan.state=state_; plan.reason=image_fresh ? "go ignored MAT_NAN_CU" : "go ignored MAT_ANH";
            plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
        }
        if(turn_fault_) barrier_done_=true;  // G khi rẽ lỗi: đi tiếp qua rào đang thấy (hết phiếu rào thì tự xoá)
        state_=!input.yolo_enabled ? PlannerState::BAM_LAN : red_latched_ ? PlannerState::DUNG_DEN_DO :
            (params.start_wait_green && !start_cleared_) ? PlannerState::CHO_DEN_XUAT_PHAT : PlannerState::BAM_LAN;
        reason_=StateName(state_); lane_.Reset(); green_run_=0; wait_since_ms_=now_ms; entered_wait=true;
        // G sau Space/reset (không phải G đầu chương trình): đồng bộ lại chặng khi hướng ổn định (R1-R6).
        if(route_g_seen_) { route_sync_=route_check_rb_=true; route_cand_ms_=-1; }
        route_g_seen_=true;
    }
    bool red=false,green=false,off=false,left=false,right=false,barrier_near=false,barrier_wide=false,barrier_seen=false,round=false,round_big=false,people=false; double red_x=0;
    bool round_right=false;  // đảo ở mép phải ảnh (cua L sau cầu)
    bool l_mask_counted=false;  // mặt nạ mới được đếm cho cua L ở bước này
    bool parking=false;  // biển parking đủ lớn trong lô này
    bool cancelled=false, sensor_cancelled=false, barrier_visible=false;
    auto fault=[&](const char *why="HUY_CAM_BIEN") {
        if(turning_) turn_end_note_=why;
        sensor_cancelled=turning_;
        if(turning_) { state_=PlannerState::BAM_LAN; lane_.Reset(); }
        if(barrier_turn_) barrier_done_=false;  // rẽ qua rào bị hủy: rào còn gần thì dừng (Q7), hồi lỗi thì rẽ lại
        ClearTurn(); turn_fault_=true; rb_pending_=0;  // không để rẽ vòng xuyến chờ bắn muộn sau khi hồi
        l_armed_=false;  // cua L đang chờ cũng bỏ (review 11/11 M1)
    };
    // Lỗi cảm biến/rẽ chỉ tạm: cảm biến tươi liên tục >= 1 s thì cho rẽ lại (IMU lỗi lúc khởi động luôn sensor_ok sai nên không hồi).
    if(params.turn_enable && turn_fault_) {
        if(sensor_bad) sensor_good_ms_=-1;
        else if(sensor_good_ms_<0) sensor_good_ms_=now_ms;
        else if(now_ms-sensor_good_ms_>=1000) { turn_fault_=false; sensor_good_ms_=-1; }
    }
    if(sensor_bad) { if(sensor_bad_since_<0) sensor_bad_since_=now_ms; } else sensor_bad_since_=-1;
    // Q9 giữ nguyên với lỗi thoáng qua; chỉ dự phòng sau đủ 250 ms lỗi liên tục.
    const bool encoder_good=input.encoder_ok && std::isfinite(input.odo_cm) && input.sensor_ms>=0 &&
        input.sensor_ms<=now_ms && now_ms-input.sensor_ms<=200;
    const bool fallback=params.imu_fallback && sensor_bad && encoder_good &&
        sensor_bad_since_>=0 && now_ms-sensor_bad_since_>=kSensorGlitchMs;
    const bool turn_ready=(!sensor_bad && !turn_fault_) || fallback;
    sensor_hold_=false;
    if(fallback && turning_ && !corridor_turn_) {
        // Không chuyển một cua IMU dở dang thành cua mù không có giao lộ.
        fault();
    }
    if(corridor_turn_ && (!params.imu_fallback || !encoder_good)) {
        ClearTurn(); state_=PlannerState::DUNG; reason_="HUY_RE_CAM_BIEN"; lane_.Reset();
    }
    if(params.turn_enable && !turn_fault_ && !fallback && !corridor_turn_) {
        if(sensor_bad) {
            // Mẫu hỏng thoáng qua: giữ nguyên lượt rẽ nhưng xe dừng (không biết chuyển động); hỏng từ kSensorGlitchMs thì hủy.
            if(now_ms-sensor_bad_since_<kSensorGlitchMs) sensor_hold_=true; else fault();
        } else if(turning_) {
            // Đang dừng vì người đi bộ: không tính thời gian đứng vào hạn rẽ/kẹt.
            if(person_stop_ && command_ms_>=0 && now_ms>command_ms_) { turn_ms_+=now_ms-command_ms_; progress_ms_=now_ms; }
            if(std::abs(input.odo_cm-turn_odo_)>params.turn_max_cm) fault("HUY_QUA_QUANG");
            else if(now_ms-turn_ms_>params.turn_max_ms) fault("HUY_QUA_GIO");
            else if(std::abs(input.odo_cm-progress_odo_)>=1) { progress_odo_=input.odo_cm; progress_ms_=now_ms; }
            else if(std::abs(input.yaw_deg-yaw0_)<params.turn_yaw_deg && now_ms-progress_ms_>=1500) fault("HUY_KET_BANH");
        }
    }
    const auto &batch=input.detections;
    const bool new_obstacle_batch=batch.frame_id>last_obstacle_frame_;
    last_obstacle_frame_=std::max(last_obstacle_frame_,batch.frame_id);
    const bool fresh=batch.frame_id && batch.captured_ms>=0 && batch.captured_ms<=now_ms &&
                     now_ms-batch.captured_ms<=params.det_max_age_ms;
    person_stop_=person_latched_;
    bool person_present=false;
    if(input.yolo_enabled) {
        if(!votes_.empty() && now_ms-votes_.back().ms>params.det_max_age_ms) green_run_=0;
        if(fresh) for(const auto &d:batch.items) {
            const bool signLeft=d.class_name=="left" || d.class_name=="no_right";
            const bool signRight=d.class_name=="right" || d.class_name=="no_left";
            const bool barrier=params.work_barrier_action && d.class_name=="work_barrier";
            const bool roundabout=d.class_name=="round_barrier";
            const bool parking_sign=params.park_enable && d.class_name=="parking";
            const bool person=d.class_name=="people" || d.class_name=="person";
            // Mọi hộp người đều giữ chốt, kể cả tin cậy thấp hoặc hộp nhỏ.
            person_present=person_present || person;
            double conf=0,area=0;
            if(d.class_name=="red") { conf=params.red_conf_min; area=params.red_box_min_area_frac; }
            else if(d.class_name=="green") { conf=params.green_conf_min; area=params.green_box_min_area_frac; }
            else if(d.class_name=="off") { conf=params.off_conf_min; area=params.off_box_min_area_frac; }
            else if(barrier || roundabout || parking_sign) conf=kBarrierConfMin;
            else if(person) conf=kPersonConfMin;
            else if(!(params.turn_enable && !turning_ && (!turn_fault_ || fallback) && (signLeft || signRight))) continue;
            const auto &b=d.box;
            if(d.captured_ms!=batch.captured_ms || !std::isfinite(d.confidence) || d.confidence<conf || d.confidence>1 ||
               !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
               b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720 ||
               double(b.width)*b.height/(1280.0*720.0)<area || ((signLeft || signRight) && b.width<kSignMinWidthFrac*1280)) continue;
            if(d.class_name=="red") { red=true; red_x=std::max(red_x,(double(b.x)+b.width)/1280.0); }
            else if(d.class_name=="green") green=true;
            else if(d.class_name=="off") off=true;
            else if(roundabout) { round=true; round_big=round_big || b.width>=kRoundMinW;  // đảo xa (hộp nhỏ) chưa mở lần qua
                round_right=round_right || (double(b.x)+b.width/2)/1280.0>=kLBarrierCx; }
            else if(person) people=people || (double(b.y)+b.height)/720.0>=kPersonStopBottom;
            else if(barrier) {
                // Đáy hộp gần xe; tâm ngoài 0,25-0,75 bề rộng là rào bên đường, không chắn trước (log 1793955267).
                const double bottom=(double(b.y)+b.height)/720.0, center=(double(b.x)+b.width/2)/1280.0;
                // Chỉ rào còn ở dải tâm mới chặn tiếp quản; rào trôi ra mép ảnh khi xe đã quay thì không.
                barrier_visible=barrier_visible || (center>=kBarrierBlockMinCx && center<=kBarrierBlockMaxCx);
                // Work 2 (hộp lệch phải) vào đường 2 làn: dải tâm tới 0,82; work 1 (hộp giữa) kích hoạt sớm hơn 0,05 theo đáy hộp.
                const bool wide=center>=kWideBarrierCx;
                barrier_seen=barrier_seen || (bottom>=params.work_barrier_y2_frac-0.10 && b.width>=(wide ? kWideBarrierSlowW : kNarrowBarrierSlowW));
                const bool near_wide=wide && bottom>=params.work_barrier_y2_frac && center<=kWideBarrierMaxCx && b.width>=kWideBarrierMinW;
                const bool near_narrow=!wide && center>=0.25 && bottom>=params.work_barrier_y2_frac-kNarrowBarrierEarly && b.width>=kNarrowBarrierMinW;
                barrier_near=barrier_near || near_wide || near_narrow;
                barrier_wide=barrier_wide || near_wide;
            }
            else if(parking_sign) parking=parking || b.width>=kParkSignMinW*1280;
            else { left=left || signLeft; right=right || signRight; }
        }
        if(!fresh || red || off || !green) green_run_=0;
        if(batch.frame_id>last_frame_) {
            last_frame_=batch.frame_id;
            if(fresh) {
                if(red) { red_ms_=batch.captured_ms; red_x_=red_x; }
                votes_.push_back({batch.captured_ms,red,green && !red && !off,left,right,barrier_near,barrier_seen,round,people,barrier_wide,round_big});
                l_right_run_=round_right ? l_right_run_+1 : 0;
                park_run_=parking ? park_run_+1 : 0;
                if(l_right_run_>=params.vote_min && !route_anchored_ && !l_done_ && std::isfinite(input.odo_cm)) { l_armed_=true; l_seen_odo_=input.odo_cm; }
                if(!entered_wait && !input.stop && image_fresh && batch.captured_ms>wait_since_ms_ && green && !red && !off &&
                   (state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO)) ++green_run_;
            }
        }
        // Phiếu theo các lô gần nhất: tuổi tính từ lô mới nhất (YOLO ~230 ms/lô, trễ ~360 ms, log 09/11),
        // lô mới nhất cũ hơn det_max_age_ms thì bỏ hết như cũ.
        const int64_t newest=votes_.empty() ? -1 : votes_.back().ms;
        votes_.erase(std::remove_if(votes_.begin(),votes_.end(),[&](const Vote& vote) {
            return vote.ms>now_ms || now_ms-newest>params.det_max_age_ms || newest-vote.ms>params.det_max_age_ms;
        }),votes_.end());
        while(votes_.size()>size_t(params.vote_window)) votes_.pop_front();
        int reds=0,greens=0,lefts=0,rights=0,barriers=0,barriers_wide=0,barriers_seen=0,rounds=0,rounds_big=0,persons=0;
        for(const auto &v:votes_) {
            reds+=v.red; greens+=v.green; lefts+=v.left; rights+=v.right; barriers+=v.barrier_near; barriers_wide+=v.barrier_wide; barriers_seen+=v.barrier_seen;
            rounds+=v.round; rounds_big+=v.round_big; persons+=v.person;
        }
        if(fresh && person_present) person_seen_ms_=std::max(person_seen_ms_,batch.captured_ms);
        if(persons>=params.vote_min) person_latched_=true;
        if(person_latched_ && fresh && !person_present && person_seen_ms_>=0 &&
           batch.captured_ms-person_seen_ms_>=kPersonReleaseMs) {
            person_latched_=false;
            for(auto &vote:votes_) vote.person=false;
        }
        person_stop_=person_latched_;
        // Lần qua vòng xuyến; quãng theo encoder (odo NaN khi encoder lỗi thì không kết thúc lần qua).
        // Mở lần qua cần hộp đảo lớn (đảo gần); đang qua thì hộp cỡ nào cũng tính, kể cả khi đảo trôi ra mép khung (log 09/11 152 s).
        const bool rb_present=rb_in_pass_ ? rounds>=params.vote_min : rounds_big>=params.vote_min;
        if(rb_present) {
            if(!rb_in_pass_ && !input.stop) { rb_entered=true; rb_in_pass_=true;  // đang dừng (Space) không mở lần qua mới
                // Lần qua mở ngay sau G-sau-Space (route_check_rb_): xe có thể đang giữa đảo nên yaw lúc này
                // không phải hướng lúc vào; không giữ hướng theo nó (log 18:01 10/11 289 s lái trái ở lối ra phải).
                rb_yaw0_=l_turning_ && turning_ ? yaw0_-90.0 : input.yaw_deg; rb_hold_odo_=input.odo_cm; rb_hold_spent_=route_check_rb_;
                if(!sensor_bad && !route_anchored_) {  // Bắc = hướng xe lúc vào đảo lần đầu (xuất phát chặng trước VX1)
                    // Đảo mở khi đang rẽ cua L sau cầu: Bắc = hướng sau cua (yaw lúc bắt đầu cua − 90°), không phải yaw giữa cua.
                    route_anchored_=true; route_yaw_ref_=l_turning_ && turning_ ? yaw0_-90.0 : input.yaw_deg; route_step_=route_q_=0;
                    route_sync_=route_ambig_=false; route_note_="NEO_VX1";
                } else if(!sensor_bad && (route_check_rb_ || route_ambig_)) {
                    // Đảo phía trước sau G: hướng Bắc chỉ có thể là đầu cả chặng (R5), hướng Nam là c4.
                    const int q=int(std::lround((input.yaw_deg-route_yaw_ref_)/90.0));
                    if(Mod4(q)==0 || Mod4(q)==2) {
                        // Log xe 11/11 (m4): 5/5 lần qua mở sau G-sau-Space không giữ hướng nên mất làn ở lối ra rồi dừng;
                        // lần qua đầu sau G thường (có giữ hướng) thoát bình thường. Nếu xe đang chỉnh theo trục lộ trình
                        // (không giữa đảo với yaw tùy ý như log 10/11 289 s) thì vẫn giữ hướng.
                        if(rb_hold_spent_ && std::abs(WrapDeg(input.yaw_deg-route_yaw_ref_-90.0*q))<=kRbHoldAlignDeg) rb_hold_spent_=false;
                        route_sync_=route_ambig_=route_check_rb_=false;
                        if(rb_turn_ && !turning_) ClearTurn();
                        if(Mod4(q)==0) { route_step_=0; route_q_=0; route_yaw_ref_=input.yaw_deg; rb_passes_=0; route_note_="SYNC_R5"; }
                        else { route_step_=4; route_q_=q; route_note_="SYNC_R4"; }
                    }
                }
                route_pass_seen_=true; }
            rb_losing_=false;
        } else if(rb_in_pass_ && (!params.imu_fallback || encoder_good)) {
            if(!rb_losing_) { rb_losing_=true; rb_lost_odo_=input.odo_cm; }
            else if(std::abs(input.odo_cm-rb_lost_odo_)>=params.rb_lost_cm) {
                rb_in_pass_=rb_losing_=false; ++rb_passes_; rb_after_odo_=input.odo_cm;
                // Lộ trình cũ (rb_pending_: lần 1 trái, lần 2 phải) đã tắt: ý định lấy từ chặng (T94). Gỡ hẳn ở giai đoạn 2.
            }
        }
        if(params.imu_fallback && !encoder_good) rb_losing_=false;
        // Bỏ ý định lộ trình chưa rẽ, KHÔNG xóa phiếu biển/rào (khác ClearTurn) để luật biển/rào bên dưới chạy như cũ.
        auto drop_route_intent=[&]() {
            if(!rb_turn_ || turning_) return;
            turn_direction_=0; rb_turn_=false; junction_votes_.clear(); last_junction_mask_ms_=-1; exit_run_=0;
        };
        if(route_anchored_) {
            if(lefts>=params.vote_min || rights>=params.vote_min || barriers_seen>=params.vote_min) {
                // H1: chặng vòng xuyến không có biển/rào; thấy đủ phiếu là đã ở chặng khác: hết lộ trình tới khi R5.
                drop_route_intent();
                if(route_step_!=kRouteSteps-1 || route_ambig_ || route_sync_) {
                    route_step_=kRouteSteps-1; route_sync_=route_ambig_=false; route_note_="HET_BIEN_RAO";
                }
            } else if(rounds>=params.vote_min) {
                drop_route_intent();  // M1: đảo phía trước (cỡ bất kỳ) thì không giữ ý định rẽ trái
                if(!sensor_bad && !rb_in_pass_ && (route_check_rb_ || route_ambig_)) {
                    const int q=int(std::lround((input.yaw_deg-route_yaw_ref_)/90.0));
                    if(Mod4(q)==0 && route_step_!=0) {  // R5 sớm: neo lại khi lần qua mở
                        route_step_=0; route_q_=q; route_sync_=route_ambig_=false; rb_passes_=0; route_note_="SYNC_R5";
                    }
                }
            }
        }
        barrier_slow=barriers_seen>=params.vote_min;
        if(fallback && params.turn_enable && params.work_barrier_action && !turn_direction_ && !barrier_done_ && !rb_holding_ && !rb_in_pass_ &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && return_cm_<0 && barriers>=params.vote_min) {
            turn_direction_=1; arm_odo_=input.odo_cm; barrier_turn_=true;
            barrier_wide_turn_=barriers_wide*2>barriers;
        }
        if(fallback && params.turn_enable && rb_pending_ && !turn_direction_ && !rb_in_pass_ && !rb_holding_ &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && return_cm_<0 &&
           std::abs(input.odo_cm-rb_after_odo_)>=(rb_pending_==1 ? params.rb_turn1_cm : params.rb_turn2_cm)) {
            turn_direction_=rb_pending_; arm_odo_=input.odo_cm; rb_pending_=0; rb_turn_=true;
        }
        if(!barriers_seen) barrier_done_=false;
        // Rào gần đủ phiếu: rẽ trái ngay bằng máy rẽ IMU (vòng kín theo yaw), chốt quyết định tới hết rẽ.
        if(params.work_barrier_action && params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && !barrier_done_ && !rb_holding_ && !rb_present &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && barriers>=params.vote_min) {
            // Rào chỉ chốt ý định rẽ trái; thời điểm rẽ do mặt nạ xác nhận giao lộ (khối giao lộ bên dưới).
            turn_direction_=1; arm_odo_=input.odo_cm; barrier_done_=true; barrier_turn_=true; barrier_wide_turn_=barriers_wide*2>barriers;
        }
        // Rào gần mà không rẽ được (lỗi cảm biến tạm hoặc IMU lỗi từ khởi động): dừng trước rào, không đi thẳng.
        // Lỗi tạm hồi (1 s tươi) thì luật trên rẽ tiếp; G sau Space bỏ qua rào này (barrier_done_).
        barrier_fault_stop=params.work_barrier_action && params.turn_enable && (turn_fault_ || sensor_bad) && !(fallback && corridor_turn_) && !barrier_done_ && !rb_present &&
            state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && barriers>=params.vote_min;
        // Lộ trình theo hướng (T94): hướng lượng tử 90° ổn định thì đếm chặng; đóng băng trong lần qua đảo.
        // M2: chỉ đếm khi xe đang chạy (không Space/dừng/chờ G) và bánh thực sự lăn trong lúc ổn định.
        const bool route_moving=!input.stop && encoder_good &&
            (state_==PlannerState::BAM_LAN || state_==PlannerState::RE_TRAI || state_==PlannerState::RE_PHAI);
        if(route_anchored_ && !sensor_bad && !rb_in_pass_ && route_moving) {
            const double rel=input.yaw_deg-route_yaw_ref_;
            const int q=int(std::lround(rel/90.0));
            if(std::abs(rel-90.0*q)>kRouteAxisDeg) route_cand_ms_=-1;
            else if(route_cand_ms_<0 || q!=route_cand_q_) { route_cand_q_=q; route_cand_ms_=now_ms; route_cand_odo_=input.odo_cm; }
            else if(now_ms-route_cand_ms_>=kRouteStableMs && std::abs(input.odo_cm-route_cand_odo_)>=kRouteMoveCm &&
                    (route_sync_ || q!=route_q_)) {
                const int old=route_step_;
                RouteStable(q);
                // Ý định lộ trình chưa rẽ chỉ thuộc chặng đã chốt nó: đổi chặng hoặc hướng lệch bảng thì bỏ.
                if(route_step_!=old || route_q_!=q || route_ambig_) drop_route_intent();
            }
        } else route_cand_ms_=-1;
        const bool route_heading_ok=route_cand_ms_>=0 && route_cand_q_==route_q_ && now_ms-route_cand_ms_>=kRouteStableMs &&
            std::abs(input.odo_cm-route_cand_odo_)>=kRouteMoveCm;
        // Ý định rẽ trái chỉ ở c2, c3 (hai ngã T trái), khi hướng khớp chặng; thời điểm rẽ vẫn do mặt nạ (M6).
        if(route_anchored_ && fresh && !route_sync_ && !route_ambig_ && (route_step_==2 || route_step_==3) && route_heading_ok &&
           params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && rounds<params.vote_min && !rb_holding_ &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE) {
            turn_direction_=1; arm_odo_=input.odo_cm; rb_turn_=true;
            left_real_ms_=-1;  // M3: chỗ mở trái của ngã vừa rẽ không tính; phải thấy lại cạnh trái thật rồi mới tính chỗ mở
        }
        // Ngã T không biển sau vòng xuyến: đi thêm rb_turn1/2_cm rồi rẽ; còn phiếu round_barrier thì không rẽ.
        if(rb_pending_ && params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && !rb_present && !rb_holding_ &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE &&
           std::abs(input.odo_cm-rb_after_odo_)>=(rb_pending_==1 ? params.rb_turn1_cm : params.rb_turn2_cm)) {
            // Sau vòng xuyến chỉ chốt ý định; rẽ khi mặt nạ thấy giao lộ, không rẽ theo quãng.
            // Lộ trình (Quân 09/11): lần 1 đi thẳng ~1 m rồi rẽ trái ngã T; lần 2 qua một cua L rồi rẽ phải ngã T kế:
            // giữ ý định tới ngã T đúng phía (cua L bị loại bởi điều kiện đường ngang), không hủy theo quãng.
            turn_direction_=rb_pending_; arm_odo_=input.odo_cm; rb_pending_=0; rb_turn_=true;
        }
        if(params.turn_enable && turn_ready && !turn_direction_ && state_==PlannerState::BAM_LAN &&
           (lefts>=params.vote_min)!=(rights>=params.vote_min)) {  // mẫu hỏng: không chốt mốc quãng NaN
            turn_direction_=lefts>=params.vote_min ? 1 : -1; arm_odo_=input.odo_cm; rb_pending_=0;
        }
        // Mốc hủy turn_arm_cm tính từ lúc biển/rào cùng chiều không còn thấy (log 09/11: biển đủ phiếu cách giao lộ 92-100 cm).
        if(turn_direction_ && !turning_ && std::isfinite(input.odo_cm) &&
           ((barrier_turn_ && barrier_seen) || (!barrier_turn_ && (turn_direction_>0 ? left : right)))) arm_odo_=input.odo_cm;
        const bool red_fresh=red_ms_>=0 && red_ms_<=now_ms && now_ms-red_ms_<=params.det_max_age_ms;
        if(red_fresh && reds>=params.vote_min && red_x_>=params.red_stop_x_frac) {
            if(!red_latched_) { green_run_=0; wait_since_ms_=now_ms; }
            red_latched_=true;
        }
        // Đang đỗ (lùi/đã đỗ): đèn không kéo xe sang DUNG_DEN_DO rồi BAM_LAN tiến khi xanh (review đỗ xe MEDIUM-1).
        if(!input.stop && state_!=PlannerState::DUNG && state_!=PlannerState::CHO_SAN_SANG &&
           state_!=PlannerState::DO_XE && state_!=PlannerState::DO_XE_XONG) {
            if(red_latched_) {
                if(turning_) { ClearTurn(); lane_.Reset(); }
                state_=PlannerState::DUNG_DEN_DO; reason_="DUNG_DEN_DO";
            }
            if((state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO) &&
               fresh && !red && !off && green && greens>=params.vote_min && reds<params.vote_min && green_run_>=params.green_frames) {
                red_latched_=false; start_cleared_=true; green_run_=0;
                votes_.clear(); red_ms_=-1; red_x_=0;
                state_=PlannerState::BAM_LAN; lane_.Reset(); released=true;
            }
        }
        if(state_==PlannerState::BAM_LAN && !released && !entered_wait && red_fresh && reds>=params.vote_min) approach_=true;
        if(!red_fresh || greens>=params.vote_min || released || entered_wait || state_!=PlannerState::BAM_LAN) approach_=false;
        if(released) reason_="DEN_XANH";
        else if(state_==PlannerState::BAM_LAN && red_fresh && approach_) reason_="TIEP_CAN_DEN_DO";
        else if(state_==PlannerState::BAM_LAN) reason_="BAM_LAN";
    }
    if(params.obstacle_action!=2 || state_!=PlannerState::BAM_LAN || entered_wait) {
        swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; swerve_odo_valid_=false; return_cm_=-1;
        if(params.obstacle_action==2 && state_!=PlannerState::DUNG_VAT_CAN) {
            obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0; ahead_clear_lots_=-1;
        }
    }
    // Quãng pha: kẹp bước encoder; mẫu hỏng/đứng 1,5 s dùng tích phân lệnh đã gửi.
    if(sensor_bad || !swerve_odo_valid_ || swerve_ms_<0 || sent_cms_<=0 ||
       swerve_progress_ms_<0 || std::abs(input.odo_cm-swerve_progress_odo_)>=1) {
        swerve_progress_odo_=input.odo_cm; swerve_progress_ms_=now_ms;
    }
    double distance=0;
    if(swerve_ms_>=0 && now_ms>swerve_ms_) {
        const double dt=(now_ms-swerve_ms_)/1000.0;
        const bool use_encoder=!sensor_bad && swerve_odo_valid_ &&
            !(sent_cms_>0 && now_ms-swerve_progress_ms_>=1500);
        distance=use_encoder ? std::min(std::abs(input.odo_cm-swerve_odo_),2*params.max_speed_cms*dt) : sent_cms_*dt;
    }
    swerve_odo_=input.odo_cm; swerve_odo_valid_=!sensor_bad;
    if(swerve_==Swerve::PASS) pass_cm_+=distance;
    if(return_cm_>=0) return_cm_+=distance;
    // Về làn phải: tối thiểu obstacle_pass_cm, rồi chờ sai số nhỏ hoặc đủ 5 lần quãng.
    if(return_cm_>=params.obstacle_pass_cm &&
       (std::abs(return_e_)<kReturnAlignedError || return_cm_>=kReturnCapFactor*params.obstacle_pass_cm)) return_cm_=-1;
    if(params.obstacle_action!=0 &&
       (state_==PlannerState::BAM_LAN || state_==PlannerState::DUNG_VAT_CAN)) {
        bool present=false, present_left=false;
        if(input.yolo_enabled && fresh && new_obstacle_batch) {
            bool obstacle=false, ahead_close=false, ahead_stop=false;
            double widest=0, widest_right=0;
            for(const auto &d:batch.items) {
                const auto &b=d.box;
                if(d.class_name!="obstacle" || d.captured_ms!=batch.captured_ms ||
                   !std::isfinite(d.confidence) || d.confidence<kObstacleConfMin || d.confidence>1 ||
                   !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
                   b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720) continue;
                present=true;
                widest=std::max(widest,double(b.width)/1280.0);
                const double cx=(double(b.x)+b.width/2)/1280.0;
                if(cx>=kHoldRightCx) widest_right=std::max(widest_right,double(b.width)/1280.0);
                const bool ahead=cx>=kPassAheadMinCx && cx<=kPassAheadMaxCx && b.width/1280.0>=params.obstacle_near_w_frac;
                ahead_close=ahead_close || (ahead && (double(b.y)+b.height)/720.0>=kPassAheadY2);
                ahead_stop=ahead_stop || (ahead && (double(b.y)+b.height)/720.0>=kPassStopY2);
                obstacle=obstacle || (b.width/1280.0>=params.obstacle_near_w_frac &&
                    (double(b.x)+b.width/2)/1280.0>=kObstacleRightLaneCx);
            }
            // Pha trái chỉ giữ bởi vật còn lớn so với vật đang né: log 10/11 18:01 vật thứ hai ở xa (nhỏ) làm pha trái
            // kéo dài 44 s / 223 cm, không bao giờ qua NE_QUA/VE_PHAI.
            // Vật trước mặt đã gần trong PASS: về làn phải ngay khi đã đi đủ phần tối thiểu của quãng pass.
            if(swerve_==Swerve::PASS && ahead_close && pass_cm_>=kPassMinFrac*params.obstacle_pass_cm)
                pass_cm_=params.obstacle_pass_cm;
            else if(swerve_==Swerve::PASS && swerve_right_seen_ && widest_right>=params.obstacle_near_w_frac && !ahead_stop) {
                // Lại có vật ở nửa phải khi đang qua/về: làn phải chưa trống, quay lại pha trái (không chuyển vào làn bị chiếm).
                swerve_=Swerve::LEFT; pass_cm_=0; obstacle_clear_run_=0; obstacle_votes_.clear();
            }
            // Log xe 11/11 (m4, 148-152 s): đang VE_PHAI mà vật 2 trước mặt (làn trái) đã sát nhưng xe vẫn chạy tới va vật;
            // trước đây chỉ PASS mới dừng. Đang về làn phải cũng dừng KHONG_DUONG_NE khi vật trước mặt sát (đáy >= 0,85).
            else if((swerve_==Swerve::PASS || return_cm_>=0) && ahead_stop) {
                state_=PlannerState::DUNG_VAT_CAN; reason_="KHONG_DUONG_NE";
                swerve_=Swerve::NONE; pass_cm_=0; return_cm_=-1; obstacle_clear_run_=0; ahead_clear_lots_=0;
            }
            // Dừng vì vật trước mặt (YOLO): chỉ nhả khi đủ vote_window lô YOLO tươi liên tiếp không còn vật gần; mặt nạ một
            // mình không nhả được (bộ đếm lỗ bỏ qua lỗ ở nửa trái đường hai làn, đúng chỗ vật này đứng; YOLO cũ thì giữ dừng).
            if(ahead_clear_lots_>=0) ahead_clear_lots_=ahead_close ? 0 : ahead_clear_lots_+1;
            if(swerve_==Swerve::LEFT) {
                swerve_peak_w_=std::max(swerve_peak_w_,widest);
                present_left=widest>0 && widest>=kSwervePresentFrac*swerve_peak_w_;
                const bool right_present=widest_right>=kSwervePresentFrac*params.obstacle_near_w_frac;
                if(right_present) swerve_right_seen_=true;
                if(swerve_right_seen_) present_left=right_present;  // đã thấy vật ở làn phải: chỉ nó giữ pha trái
            }
            if(state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && return_cm_<0 &&
               !rb_holding_ && !rb_in_pass_ && !turn_direction_) {
                obstacle_votes_.push_back(obstacle);
                while(obstacle_votes_.size()>size_t(params.vote_window)) obstacle_votes_.pop_front();
                if(std::count(obstacle_votes_.begin(),obstacle_votes_.end(),true)>=params.vote_min) {
                    obstacle_votes_.clear(); obstacle_clear_run_=0;
                    if(params.obstacle_action==1) { state_=PlannerState::DUNG_VAT_CAN; reason_="VAT_CAN"; }
                    else { swerve_=Swerve::LEFT; pass_cm_=0; swerve_peak_w_=widest; present_left=true; swerve_right_seen_=false; }
                }
            } else obstacle_votes_.clear();
        }
        if(mask_fresh && (input.mask_ms!=last_mask_ms_ || swerve_==Swerve::LEFT)) {
            last_mask_ms_=input.mask_ms;
            bool near_hole=false, avoid_ok=true, mask_obstacle=false;
            if(!rb_in_pass_) {
                const auto corridor=ExtractCorridor(input.mask,params,TrajMode::LANE);
                const double bottom=std::lround(params.corridor_bottom_frac*input.mask.rows);
                const double top=std::lround(params.corridor_top_frac*input.mask.rows);
                int candidates=0;
                for(const auto &row:corridor.rows) {
                    if(!row.valid || !row.hole) continue;
                    // Đường 2 làn: lỗ nằm trọn nửa trái là vật trên làn trái, không chắn làn phải (vật thứ hai sau khi về làn).
                    if(row.lanes==2 && row.hole_r<=(row.left+row.right)/2) continue;
                    const double t=bottom>top ? std::max(0.0,std::min(1.0,(bottom-row.y)/(bottom-top))) : 0;
                    const double w=input.mask.cols*(params.lane1_width_frac[0]+(params.lane1_width_frac[1]-params.lane1_width_frac[0])*t);
                    // hole_l/r là các pixel 0, hai đoạn đường hai phía đã được ExtractCorridor chọn.
                    // Lỗ hẹp hơn ngưỡng vật cản là nhiễu: không tính vật cản, không dùng để kết luận không có đường né (log 09/11 350 s: lỗ 3 px).
                    if(row.hole_r-row.hole_l+1<params.obstacle_hole_ratio*w) continue;
                    // Chỉ nửa gần: lỗ hàng xa cho báo giả (log 09/11 112,7 s) mà không nhận vật thật sớm hơn luật thụt mép (phát lại 2 phiên).
                    if(row.y<(params.corridor_bottom_frac+params.corridor_top_frac)*input.mask.rows/2) continue;
                    near_hole=true; avoid_ok=avoid_ok && row.avoid_ok;
                    ++candidates;
                }
                // Vật chiếm trọn làn phải trên đường 2 làn không tạo lỗ: mép phải hàng trên thụt vào >= 0,5 làn so với hàng dưới.
                // Bỏ hàng xa nhất (đường cụt ngã T) và lúc đã có ý định rẽ (log 09/11: 3/446 mask, 2 báo giả ở ngã T có biển).
                bool intrusion=false;
                // Phải thấy ở 2 hàng liền nhau (hàng thụt vào và hàng ngay trên vẫn thụt) để bỏ báo giả một hàng.
                for(size_t i=1;i+1<corridor.rows.size() && !turn_direction_;++i) {
                    const auto &a=corridor.rows[i-1], &b=corridor.rows[i], &c=corridor.rows[i+1];
                    if(!a.valid || !b.valid || !c.valid) continue;
                    const double t=bottom>top ? std::max(0.0,std::min(1.0,(bottom-b.y)/(bottom-top))) : 0;
                    const double w=input.mask.cols*(params.lane1_width_frac[0]+(params.lane1_width_frac[1]-params.lane1_width_frac[0])*t);
                    intrusion=intrusion || (a.lanes==2 && a.flag_right=="THAT" && b.flag_left=="THAT" && b.flag_right!="MEP_ANH" &&
                        a.raw_r-b.raw_r>=0.5*w && b.raw_r-b.raw_l<=1.25*w && c.flag_right!="MEP_ANH" && a.raw_r-c.raw_r>=0.5*w);
                }
                mask_obstacle=candidates>=2 || intrusion;
            }
            if(state_==PlannerState::DUNG_VAT_CAN && reason_=="KHONG_DUONG_NE") {
                obstacle_clear_run_=mask_obstacle || (near_hole && !avoid_ok) ? 0 : obstacle_clear_run_+1;
                if(obstacle_clear_run_>=params.vote_window && (ahead_clear_lots_<0 || ahead_clear_lots_>=params.vote_window)) {
                    state_=PlannerState::BAM_LAN; reason_="BAM_LAN"; lane_.Reset(); ahead_clear_lots_=-1;
                    obstacle_votes_.clear(); obstacle_clear_run_=0;
                }
            }
            if(params.obstacle_action==2 && swerve_==Swerve::LEFT && near_hole && !avoid_ok) {
                state_=PlannerState::DUNG_VAT_CAN; reason_="KHONG_DUONG_NE";
                swerve_=Swerve::NONE; pass_cm_=0; return_cm_=-1; obstacle_clear_run_=0;
            }
        }
        if(input.yolo_enabled && fresh && new_obstacle_batch) {
            if(params.obstacle_action==2 && swerve_==Swerve::LEFT) {
                obstacle_clear_run_=present_left ? 0 : obstacle_clear_run_+1;
                if(obstacle_clear_run_>=params.vote_window) {
                    swerve_=Swerve::PASS; pass_cm_=0; obstacle_votes_.clear();
                }
            } else if(state_==PlannerState::DUNG_VAT_CAN && reason_!="KHONG_DUONG_NE") {
                obstacle_clear_run_=present ? 0 : obstacle_clear_run_+1;
                if(obstacle_clear_run_>=params.vote_window) {
                    state_=PlannerState::BAM_LAN; reason_="BAM_LAN"; lane_.Reset();
                    obstacle_votes_.clear(); obstacle_clear_run_=0;
                }
            }
        }
    } else {
        obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0; ahead_clear_lots_=-1;
    }
    if(swerve_==Swerve::PASS && pass_cm_>=params.obstacle_pass_cm) {
        swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0; obstacle_votes_.clear();
        lane_.Reset(); return_cm_=0; return_e_=1;
    }
    // Phiếu giao lộ độc lập với phiếu vật cản, chỉ thêm một lần cho mỗi mask mới.
    // Theo dõi cạnh thật cả trước khi chốt ý định (rào 2: cạnh trái đã mở ngay lúc chốt, log 09/11 sáng 65 s).
    if(params.turn_enable && (!turning_ || corridor_turn_) && !rb_in_pass_ && mask_fresh && image_fresh && !input.stop &&
       input.mask_ms!=last_junction_mask_ms_) {
        last_junction_mask_ms_=input.mask_ms;
        const auto corridor=ExtractCorridor(input.mask,params,TrajMode::LANE);
        // (a) Cạnh phía rẽ ở nửa hàng gần chuyển từ cạnh thật sang mở (nhánh rẽ đã tới sát xe);
        //     đường 2 làn có cạnh trái ngoài khung từ đầu thì không tính (chưa từng thấy cạnh thật).
        // (b) Đường phía trước kết thúc gần (ngã T). Log 09/11: cả hai đến muộn hơn lúc rẽ sớm 40-60 cm.
        int real_left=0, real_right=0, valid_near=0;
        const size_t near_rows=std::min(corridor.rows.size(),size_t(params.corridor_rows/2));
        for(size_t i=0;i<near_rows;++i) {
            const auto &row=corridor.rows[i];
            if(!row.valid) continue;
            ++valid_near; real_left+=row.flag_left=="THAT"; real_right+=row.flag_right=="THAT";
        }
        int open_both=0;
        for(size_t i=0;i<near_rows;++i) open_both+=corridor.rows[i].valid && corridor.rows[i].flag_left=="MEP_ANH" && corridor.rows[i].flag_right=="MEP_ANH";
        l_open_run_=open_both>=kLOpenRows ? l_open_run_+1 : 0; l_mask_counted=true;
        if(real_left>=2) left_real_ms_=now_ms;
        if(real_right>=2) right_real_ms_=now_ms;
        const int opened=valid_near-(turn_direction_>0 ? real_left : real_right);
        const int64_t real_ms=turn_direction_>0 ? left_real_ms_ : right_real_ms_;
        const bool real_before=real_ms>=0 && now_ms-real_ms<=kRealSideMs;
        int cx=input.mask.cols/2;
        if(!corridor.rows.empty() && corridor.rows[0].valid) cx=int(std::lround((corridor.rows[0].raw_l+corridor.rows[0].raw_r)/2));
        cx=std::max(4,std::min(input.mask.cols-5,cx));
        int gap=0, road_end=-1;
        for(int y=int(std::lround(params.corridor_bottom_frac*input.mask.rows));y>=0 && road_end<0;--y) {
            const auto *px=input.mask.ptr<unsigned char>(y);
            int on=0; for(int x=cx-4;x<=cx+4;++x) on+=px[x]!=0;
            gap=on<3 ? gap+1 : 0;
            if(gap>=3) road_end=y+2;  // 3 hàng liền không có đường: mép xa của đường phía trước
        }
        // Ngã T: ngay trước chỗ đường kết thúc, đường ngang phải kéo dài về phía rẽ tới gần mép ảnh.
        // Cua chữ L chỉ mở một phía: ý định rẽ ngược phía cua thì không tính (lộ trình sau vòng xuyến lần 2 có cua L trước ngã T).
        bool side_road=false;
        if(turn_direction_ && road_end>=0 && road_end>=kRoadEndFrac*input.mask.rows) {
            const int yc=std::min(input.mask.rows-1,road_end+3);
            const auto *px=input.mask.ptr<unsigned char>(yc);
            int x=cx;
            if(turn_direction_>0) { while(x>0 && px[x-1]) --x; side_road=x<=kSideRoadEdgeFrac*input.mask.cols; }
            else { while(x<input.mask.cols-1 && px[x+1]) ++x; side_road=x>=(1-kSideRoadEdgeFrac)*input.mask.cols; }
        }
        const bool junction=turn_direction_ && ((opened>=2 && real_before) || side_road);
        if(turn_direction_) junction_votes_.push_back(junction);
        if(corridor_turn_) {
            const auto selected=ExtractCorridor(input.mask,params,turn_direction_>0 ? TrajMode::TURN_LEFT : TrajMode::TURN_RIGHT);
            int usable=0;
            bool real=false;
            for(const auto &row:selected.rows) if(row.valid && row.conf>=params.conf_min) {
                ++usable; real=real || row.flag_left=="THAT" || row.flag_right=="THAT";
            }
            corridor_clear_=!junction && real && usable>=std::max(2,params.min_valid_rows) ? corridor_clear_+1 : 0;
        }
        while(junction_votes_.size()>size_t(params.vote_window)) junction_votes_.pop_front();
    }
    // Rào phải có lô YOLO tươi xác nhận vắng rào; hai nguồn còn lại giữ luật đường ra.
    const bool source_clear=barrier_turn_ ? fresh && !barrier_visible :
        rb_turn_ ? !rb_in_pass_ : turn_direction_!=0;
    if(!source_clear) exit_run_=0; // Lô rào mới cũng vô hiệu phiếu dù mặt nạ chưa đổi.
    // Hành lang đường ra khi đang rẽ IMU: đủ hàng 2 cạnh thật và tâm hành lang gần trục xe, liên tiếp vote_min mặt nạ.
    if(turning_ && !corridor_turn_ && mask_fresh && input.mask_ms!=last_exit_mask_ms_) {
        last_exit_mask_ms_=input.mask_ms;
        const auto corridor=ExtractCorridor(input.mask,params,TrajMode::LANE);
        int real_rows=0; double offset=0;
        for(const auto &row:corridor.rows) if(row.valid && row.conf>=params.conf_min && row.flag_left=="THAT" && row.flag_right=="THAT") {
            ++real_rows; offset+=((row.left+row.right)/2-input.mask.cols/2.0)/input.mask.cols;
        }
        const bool exit_ok=source_clear && real_rows>=kExitRealRows && std::abs(offset/std::max(1,real_rows))<=kExitOffsetFrac;
        exit_run_=exit_ok ? exit_run_+1 : 0;
    }
    const bool junction_seen=!rb_in_pass_ && std::count(junction_votes_.begin(),junction_votes_.end(),true)>=params.vote_min;
    const char *turn_start_reason=nullptr;
    const bool returning=return_cm_>=0;
    // Lịch sử yaw theo quãng khi đang bám làn có ý định rẽ (chưa rẽ): dùng đo phần đã quay sẵn lúc bắt đầu RE_*.
    if(turn_direction_ && !turning_ && state_==PlannerState::BAM_LAN && !sensor_bad &&
       std::isfinite(input.odo_cm) && std::isfinite(input.yaw_deg)) {
        if(!pre_turn_.empty() && input.odo_cm<pre_turn_.back().first) pre_turn_.clear();
        if(pre_turn_.empty() || input.odo_cm-pre_turn_.back().first>=0.5) pre_turn_.emplace_back(input.odo_cm,input.yaw_deg);  // đứng yên không thêm mẫu
        while(pre_turn_.size()>1 && input.odo_cm-pre_turn_[1].first>=kPreTurnWindowCm) pre_turn_.pop_front();
    } else if(!turning_) pre_turn_.clear();
    if(!l_mask_counted && (turning_ || rb_in_pass_ || input.stop || !mask_fresh)) l_open_run_=0;  // review LOW-1: không dùng đếm cũ
    if(l_armed_ && (route_anchored_ || rb_in_pass_ || !std::isfinite(input.odo_cm) || std::abs(input.odo_cm-l_seen_odo_)>kLArmCm)) l_armed_=false;
    if(params.turn_enable && l_armed_ && !l_done_ && !turning_ && !turn_direction_ && state_==PlannerState::BAM_LAN &&
       swerve_==Swerve::NONE && !returning && !rb_holding_ && !sensor_bad && mask_fresh && !input.stop &&
       !turn_fault_ && l_mask_counted && std::isfinite(input.yaw_deg) && l_open_run_>=params.vote_min) {
        l_armed_=false; l_done_=true; l_turning_=true; turn_direction_=-1; arm_odo_=input.odo_cm;
        turn_start_reason="RE_L_CAU";
        turning_=true; corridor_turn_=false; corridor_clear_=0; turn_steer_scale_=1;
        yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm; turn_ms_=progress_ms_=now_ms; turn_end_note_.clear();
        state_=PlannerState::RE_PHAI;
    }
    if(params.turn_enable && turn_direction_ && state_!=PlannerState::DUNG && state_!=PlannerState::DUNG_DEN_DO &&
       state_!=PlannerState::DUNG_VAT_CAN && state_!=PlannerState::DO_XE && state_!=PlannerState::DO_XE_XONG) {
        if(corridor_turn_ && (std::abs(input.odo_cm-turn_odo_)>params.turn_max_cm || now_ms-turn_ms_>params.turn_max_ms)) {
            ClearTurn(); cancelled=true; state_=PlannerState::DUNG; reason_="RE_HANH_LANG_HET_HAN"; lane_.Reset();
        } else if(corridor_turn_ && corridor_clear_>=params.vote_min) {
            ClearTurn(); state_=PlannerState::BAM_LAN; lane_.Reset();
        } else if(!mask_fresh) {
            if(barrier_turn_) barrier_done_=false;  // rẽ rào bị hủy vì mặt nạ cũ: rào còn gần thì xét lại (rẽ hoặc dừng Q7)
            ClearTurn(); state_=PlannerState::BAM_LAN;
        } else if(turning_ && !corridor_turn_ && !sensor_bad && std::isfinite(input.yaw_deg) &&
                  std::abs(WrapDeg(input.yaw_deg-yaw0_))>=params.turn_yaw_deg*(turn_direction_<0 ? kRightTurnYawFactor : 1.0)-kHandoverTolDeg &&
                  exit_run_>=params.vote_min) {  // giao bám làn: IMU đã quay gần tới đích VÀ mặt nạ xác nhận đường ra (cả hai)
            turn_end_note_="XONG_IMU_VA_MASK"; ClearTurn(); state_=PlannerState::BAM_LAN; lane_.Reset();
        } else if(turning_ && !corridor_turn_ && !sensor_bad && std::abs(WrapDeg(input.yaw_deg-yaw0_))>=params.turn_yaw_deg*(turn_direction_<0 ? kRightTurnYawFactor : 1.0)) {  // mẫu hỏng (kể cả ±inf) không được coi là đã quay đủ
            turn_end_note_=exit_run_>=params.vote_min ? "XONG_IMU_VA_MASK" : "XONG_IMU_CHUA_THAY_LAN"; ClearTurn(); state_=PlannerState::BAM_LAN; lane_.Reset();
        } else if(!turning_) {
            if(!rb_turn_ && (!sensor_bad || fallback) && std::abs(input.odo_cm-arm_odo_)>params.turn_arm_cm) { ClearTurn(); cancelled=true; }
            // Chỉ bắt đầu rẽ khi mặt nạ xác nhận giao lộ (không rẽ theo quãng ước lượng); đang né/về làn thì chờ bám làn thường.
            else if(state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && !returning && !rb_holding_ && (!sensor_bad || fallback) &&
                    !(rb_in_pass_ && !rb_losing_) &&  // còn đủ phiếu vòng xuyến thì chưa rẽ (đảo phía trước)
                    junction_seen) {
                turn_start_reason="RE_GIAO_LO";
                turning_=true; corridor_turn_=fallback; corridor_clear_=0;
                if(corridor_turn_ && barrier_turn_) barrier_done_=true;
                yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm; turn_end_note_.clear();
                const double pre=pre_turn_.empty() || !std::isfinite(input.yaw_deg) ? 0 :
                    std::max(0.0,turn_direction_*WrapDeg(input.yaw_deg-pre_turn_.front().second));
                // Chỉ rẽ phải theo biển/giao lộ có bằng chứng (6 lượt); rẽ trái, rào, vòng xuyến giữ góc cũ (review 11/11 MEDIUM-1).
                turn_steer_scale_=turn_direction_<0 && !barrier_turn_ && !rb_turn_ ?
                    std::max(kPreTurnMinScale,std::min(1.0,1-pre/kPreTurnRefDeg)) : 1.0;
                turn_ms_=progress_ms_=now_ms;
                state_=turn_direction_==1 ? PlannerState::RE_TRAI : PlannerState::RE_PHAI;
            }
        }
    }
    // Đỗ xe lùi song song (chặng 8). Cần turn_enable (yaw/encoder chỉ được đọc khi bật rẽ).
    if(!params.park_enable || !params.turn_enable) {
        if(park_phase_==ParkPhase::TIEN) park_phase_=ParkPhase::NONE;
    } else {
        if(park_phase_==ParkPhase::NONE && state_==PlannerState::BAM_LAN && !turning_ && !turn_direction_ &&
           swerve_==Swerve::NONE && !returning && !rb_in_pass_ && !rb_holding_ && !sensor_bad && park_run_>=params.vote_min) {
            park_phase_=ParkPhase::TIEN; park_odo0_=input.odo_cm;
            park_mode_=0; park_unknown_=0; park_step_cm_=params.park_forward_cm; park_scan_cm_=park_creep_cm_=0; park_cap_cms_=0;
            park_note_="BIEN_P_VAO_BAI";
        }
        // Có ý định rẽ trong lúc tiến (biển/lộ trình): bỏ đỗ, để rẽ chạy như thường (review MEDIUM-2).
        if(park_phase_==ParkPhase::TIEN && (state_!=PlannerState::BAM_LAN || turning_ || turn_direction_ || sensor_bad ||
                                            rb_in_pass_ || rb_holding_)) park_phase_=ParkPhase::NONE;
        if(park_phase_==ParkPhase::TIEN && std::abs(input.odo_cm-park_odo0_)>=park_step_cm_) {
            ClearTurn(); park_ms0_=now_ms; state_=PlannerState::DO_XE;
            if(park_mode_==2) { park_phase_=ParkPhase::CHO; reason_="DO_XE_CHO"; park_note_="DUNG_BAT_DAU_LUI"; }
            else { park_phase_=ParkPhase::QUET; park_scan_ms0_=now_ms; reason_="DO_XE_QUET"; }
        }
    }
    if(state_==PlannerState::DO_XE || state_==PlannerState::DO_XE_XONG) {
        auto stop_park=[&](const char *why) { park_phase_=ParkPhase::NONE; state_=PlannerState::DUNG; reason_=why; };
        auto start_phase=[&](ParkPhase phase, const char *why) {
            park_phase_=phase; park_odo0_=park_prog_odo_=input.odo_cm; park_ms0_=park_prog_ms_=now_ms; reason_=why;
        };
        const bool reversing=park_phase_==ParkPhase::LUI1 || park_phase_==ParkPhase::LUI2;
        if(sensor_bad && park_phase_!=ParkPhase::XONG) stop_park("DO_XE_CAM_BIEN");
        else if(park_phase_==ParkPhase::QUET) {
            // Xe đứng: đợi ổn định, rồi chỉ nhận mẫu ToF lấy SAU thời điểm đó; không chắc thì dừng, không đỗ mù.
            const bool left=params.park_side==1;
            const TofSlot &sf=input.tof[left ? 1 : 2], &sr=input.tof[left ? 0 : 3];
            const int64_t min_ms=park_scan_ms0_+kParkSettleMs, waited=now_ms-park_scan_ms0_;
            ScanCell cf=ClassifyTofSample(sf,now_ms,kParkOccMm,kParkTofAgeMs), cr=ClassifyTofSample(sr,now_ms,kParkOccMm,kParkTofAgeMs);
            if(sf.sample_ms<min_ms) cf=ScanCell::UNKNOWN;
            if(sr.sample_ms<min_ms) cr=ScanCell::UNKNOWN;
            const bool fresh=sf.sample_ms>=min_ms || sr.sample_ms>=min_ms;
            if(person_stop_ && command_ms_>=0 && now_ms>command_ms_) park_scan_ms0_+=now_ms-command_ms_;
            if(waited<kParkSettleMs || (!fresh && waited<kParkScanMaxMs)) {
                reason_="DO_XE_QUET";
            } else if(!fresh) { park_note_="TOF_KHONG_CO_MAU"; stop_park("DO_XE_TOF_KHONG_RO"); }
            else {
                auto name=[](ScanCell c) { return c==ScanCell::EMPTY ? "TRONG" : c==ScanCell::OCCUPIED ? "CO_VAT" : "KHONG_RO"; };
                auto step=[&](double cm, double cap, int mode) {
                    park_phase_=ParkPhase::TIEN; park_odo0_=input.odo_cm; park_step_cm_=cm; park_cap_cms_=cap; park_mode_=mode;
                    state_=PlannerState::BAM_LAN; reason_="BAM_LAN"; park_ms0_=now_ms;
                };
                const ScanCell both=cf==cr ? cf : ScanCell::UNKNOWN;
                park_note_=std::string("mode=")+std::to_string(park_mode_)+" truoc="+name(cf)+"("+std::to_string(sf.mm)+"mm) sau="+name(cr)+"("+std::to_string(sr.mm)+"mm) gop="+name(both);
                if(park_mode_==0) {
                    if(both==ScanCell::EMPTY) { park_unknown_=0; park_creep_cm_=0; step(kParkCreepCm,kParkCreepCms,1); park_note_+=" -> O_TRONG_BO_TIM_MEP"; }
                    else {
                        park_unknown_=both==ScanCell::UNKNOWN ? park_unknown_+1 : 0;
                        park_scan_cm_+=kParkStepCm;
                        if(park_unknown_>=2) stop_park("DO_XE_TOF_KHONG_RO");
                        else if(park_scan_cm_>=kParkMaxScanCm) stop_park("DO_XE_KHONG_CO_O");
                        else step(kParkStepCm,kParkStepCms,0);
                    }
                } else {  // mode 1: bò từng 10 cm tới khi cảm biến SAU thấy vật ở ô kế (mép xa của ô trống)
                    if(cr==ScanCell::OCCUPIED) { step(kParkPastCm,kParkCreepCms,2); park_note_+=" -> THAY_MEP_XA"; }
                    else {
                        park_unknown_=cr==ScanCell::UNKNOWN ? park_unknown_+1 : 0;
                        park_creep_cm_+=kParkCreepCm;
                        if(park_unknown_>=2) stop_park("DO_XE_TOF_KHONG_RO");
                        else if(park_creep_cm_>=kParkMaxCreepCm) stop_park("DO_XE_KHONG_THAY_MEP");
                        else step(kParkCreepCm,kParkCreepCms,1);
                    }
                }
            }
        } else if(park_phase_==ParkPhase::CHO) {
            if(now_ms-park_ms0_>=kParkWaitMs) { park_yaw0_=input.yaw_deg; start_phase(ParkPhase::LUI1,"DO_XE_LUI_1"); }
        } else if(reversing) {
            // Lùi + lái về phía ô: đuôi vào ô; quay đo theo chiều đó (lùi + lái phải -> yaw tăng).
            const double turned=-params.park_side*WrapDeg(input.yaw_deg-park_yaw0_);
            if(person_stop_ && command_ms_>=0 && now_ms>command_ms_) {  // đứng vì người không tính vào hạn
                park_ms0_+=now_ms-command_ms_; park_prog_ms_=now_ms;
            }
            if(std::abs(input.odo_cm-park_prog_odo_)>=1) { park_prog_odo_=input.odo_cm; park_prog_ms_=now_ms; }
            if(std::abs(input.odo_cm-park_odo0_)>kParkPhaseMaxCm || now_ms-park_ms0_>kParkPhaseMaxMs) stop_park("DO_XE_HET_HAN");
            else if(!person_stop_ && now_ms-park_prog_ms_>=kParkStallMs) stop_park("DO_XE_KET");
            else if(park_phase_==ParkPhase::LUI1 && turned>=params.park_arc_deg) start_phase(ParkPhase::LUI2,"DO_XE_LUI_2");
            else if(park_phase_==ParkPhase::LUI2 && turned<=kParkYawDoneDeg) {
                park_phase_=ParkPhase::XONG; state_=PlannerState::DO_XE_XONG; reason_="DO_XE_XONG";
            }
        }
        if(state_==PlannerState::DO_XE || state_==PlannerState::DO_XE_XONG) {
            double cms=0, target=0;
            if(park_phase_==ParkPhase::LUI1 || park_phase_==ParkPhase::LUI2) {
                cms=-std::min(params.park_rev_cms,params.max_speed_cms);
                target=(park_phase_==ParkPhase::LUI1 ? 1 : -1)*params.park_side*std::min(params.park_steer_deg,params.steer_max_deg);
            }
            if(person_stop_) { cms=0; target=sent_angle_; }
            const double step=params.steer_rate_dps*(command_ms_<0 ? 0 : std::max<int64_t>(0,now_ms-command_ms_)/1000.0);
            sent_angle_=std::max(-params.steer_max_deg,std::min(params.steer_max_deg,
                std::max(sent_angle_-step,std::min(sent_angle_+step,target))));
            command_ms_=now_ms;
            plan.state=state_; plan.cmd={cms,sent_angle_};
            plan.reason=person_stop_ ? "NGUOI_DI_BO" : reason_;
            plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
            plan.debug.angle_deg=sent_angle_; plan.debug.speed_cms=cms;
            return plan;
        }
    }
    plan.state=state_;
    if(params.turn_enable && corridor_turn_ && turning_ && (state_==PlannerState::RE_TRAI || state_==PlannerState::RE_PHAI)) {
        plan.mode=turn_direction_>0 ? TrajMode::TURN_LEFT : TrajMode::TURN_RIGHT;
        const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,plan.mode);
        plan.debug=result.debug; plan.cmd=result.cmd;
        plan.cmd.cms=std::min(plan.cmd.cms,params.min_cms);
        if(person_stop_) plan.cmd.cms=0;
        plan.reason=person_stop_ ? "NGUOI_DI_BO" : TrajModeName(plan.mode);
        if(result.debug.state!="BAM_LAN") plan.reason=result.debug.reason;
        plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
        plan.debug.speed_cms=plan.cmd.cms; plan.debug.angle_deg=plan.cmd.deg;
        sent_angle_=plan.cmd.deg; command_ms_=now_ms;
        return plan;
    }
    if(params.turn_enable && turning_ && (state_==PlannerState::RE_TRAI || state_==PlannerState::RE_PHAI)) {
        const double step=sensor_hold_ ? 0 : params.steer_rate_dps*(command_ms_<0 ? 0 : std::max<int64_t>(0,now_ms-command_ms_)/1000.0);  // giữ nguyên góc khi mẫu hỏng
        const double base=TurnSteer(params,turn_direction_,barrier_turn_,barrier_wide_turn_)*turn_steer_scale_;
        // IMU là phản hồi chính: yaw còn thiếu so với quãng còn lại quyết định góc lái (chỉ tăng khi chậm tiến độ, trần steer_max_deg).
        turn_target_deg_=params.turn_yaw_deg*(turn_direction_<0 ? kRightTurnYawFactor : 1.0);
        const double done=std::isfinite(input.yaw_deg) ? std::abs(WrapDeg(input.yaw_deg-yaw0_)) : 0.0;
        turn_err_deg_=turn_target_deg_-done;
        double need=0;
        if(turn_err_deg_>0 && !sensor_hold_) {
            const double rem_cm=std::max(5.0,kTurnPaceMargin*(params.turn_max_cm-std::abs(input.odo_cm-turn_odo_)));
            // turn_steer_scale_ < 1: xe đã quay sẵn về phía rẽ (lỗi 2), phần còn lại nhỏ hơn đích đầy đủ.
            const double remaining=std::max(0.0,turn_err_deg_-(1.0-turn_steer_scale_)*turn_target_deg_);
            need=std::atan(kTurnLeffCm*(remaining*3.14159265358979323846/180.0)/rem_cm)*180.0/3.14159265358979323846;
        }
        const double target=turn_direction_*std::min(params.steer_max_deg,std::max(base,need));
        sent_angle_=std::max(-params.steer_max_deg,std::min(params.steer_max_deg,
            std::max(sent_angle_-step,std::min(sent_angle_+step,target))));
        command_ms_=now_ms;
        plan.cmd={person_stop_ || sensor_hold_ ? 0.0 : std::min(params.max_speed_cms,std::max(params.min_cms,kTurnCms)),sent_angle_};
        plan.reason=person_stop_ ? "NGUOI_DI_BO" : sensor_hold_ ? "CAM_BIEN_CHO" : turn_start_reason ? turn_start_reason : StateName(state_);
        plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
        plan.debug.angle_deg=sent_angle_; plan.debug.speed_cms=plan.cmd.cms;
        return plan;
    }
    if(state_!=PlannerState::BAM_LAN) {
        sent_angle_=0; command_ms_=now_ms;
        plan.reason=reason_; plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
    }
    plan.mode=swerve_!=Swerve::NONE ? TrajMode::AVOID_LEFT : returning ? TrajMode::RETURN_RIGHT : rb_in_pass_ ? TrajMode::ROUNDABOUT : TrajMode::LANE;
    const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,plan.mode,turn_direction_==0);
    if(returning && result.debug.state=="BAM_LAN") return_e_=result.debug.e;  // mất làn thì giữ sai số cũ, không kết thúc giả
    plan.cmd=result.cmd; plan.debug=result.debug; plan.reason=result.debug.reason;
    // Vòng xuyến (phương án B): mất làn trong một lần qua thì lái giữ hướng lúc vào thay vì dừng,
    // tới khi bắt lại làn, hết lần qua, quá rb_hold_max_cm hoặc cảm biến lỗi (khi đó dừng như cũ).
    const bool was_holding=rb_holding_;
    rb_holding_=false;
    if(rb_in_pass_ && result.debug.state!="BAM_LAN" && result.debug.state!="MAT_NAN_CU" && mask_fresh && swerve_==Swerve::NONE &&
       params.turn_enable && !turn_fault_ && !sensor_bad && std::isfinite(rb_yaw0_) && !rb_hold_spent_) {
        if(!was_holding) { rb_hold_ms_=rb_prog_ms_=now_ms; rb_prog_odo_=input.odo_cm; }
        if(std::abs(input.odo_cm-rb_prog_odo_)>=1) { rb_prog_odo_=input.odo_cm; rb_prog_ms_=now_ms; }
        // Giới hạn như máy rẽ: quá quãng, quá turn_max_ms, hoặc bánh không tiến 1 cm trong 1500 ms (kẹt đảo/rào).
        if(std::abs(input.odo_cm-rb_hold_odo_)>params.rb_hold_max_cm || now_ms-rb_hold_ms_>params.turn_max_ms ||
           now_ms-rb_prog_ms_>=1500) rb_hold_spent_=true;
        else {
            rb_holding_=true;
            const double target=std::max(-params.steer_max_deg,std::min(params.steer_max_deg,
                params.rb_yaw_kp*WrapDeg(rb_yaw0_-input.yaw_deg)));
            const double step=params.steer_rate_dps*(command_ms_<0 ? 0 : std::max<int64_t>(0,now_ms-command_ms_)/1000.0);
            const double angle=std::max(sent_angle_-step,std::min(sent_angle_+step,target));
            plan.cmd={std::min(params.max_speed_cms,std::max(params.min_cms,params.cruise_cms-params.slow_per_deg*std::abs(angle))),angle};
            plan.state=PlannerState::GIU_HUONG; plan.reason="GIU_HUONG";
            plan.debug.state=plan.debug.reason="GIU_HUONG"; plan.debug.angle_deg=angle; plan.debug.speed_cms=plan.cmd.cms;
        }
    }
    if(plan.reason=="BAM_LAN" && rb_in_pass_ && swerve_==Swerve::NONE && !returning)
        plan.reason=plan.debug.reason=rb_losing_ ? "VX_THOAT" : fallback && rb_entered ? "VX_TIEP_CAN" : "VX_QUANH_DAO";
    if(input.yolo_enabled && (!fresh || reason_=="TIEP_CAN_DEN_DO" || released)) {
        // Quân 11/11: tiếp cận đèn đỏ bằng tốc độ bám làn (cruise); approach_cms chỉ còn kẹp khi mất YOLO (!fresh).
        if(!fresh) plan.cmd.cms=std::min({plan.cmd.cms,params.approach_cms,params.max_speed_cms});
        if(result.debug.state=="BAM_LAN") {
            plan.reason=released ? "DEN_XANH" : !fresh ? "MAT_YOLO" : reason_; plan.debug.reason=plan.reason;
        }
        plan.debug.speed_cms=plan.cmd.cms;
    }
    if(park_phase_==ParkPhase::TIEN && park_cap_cms_>0) {  // tiến từng bước quét ô: chậm để dừng đúng vị trí
        plan.cmd.cms=std::min(plan.cmd.cms,park_cap_cms_); plan.debug.speed_cms=plan.cmd.cms;
    }
    if(barrier_slow && result.debug.state=="BAM_LAN") {  // thấy rào phía trước: giảm tốc trước khi rẽ
        plan.cmd.cms=std::min({plan.cmd.cms,params.approach_cms,params.max_speed_cms}); plan.debug.speed_cms=plan.cmd.cms;
    }
    // Người đi bộ: dừng trước khi cộng quãng né (sent_cms_), state giữ nguyên (cả khi giữ hướng vòng xuyến).
    const bool person_hold=person_stop_;
    if(person_hold || barrier_fault_stop) plan.cmd.cms=0;  // trước khi cộng quãng né/về làn (sent_cms_)
    if(params.obstacle_action==2) {
        if(result.debug.state!="BAM_LAN") {
            if(swerve_!=Swerve::NONE) {
                state_=PlannerState::DUNG; reason_="MAT_LAN_NE"; lane_.Reset(); green_run_=0;
                swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; swerve_odo_valid_=false; return_cm_=-1;
                obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0; ahead_clear_lots_=-1;
                plan=Plan{}; plan.state=state_; plan.reason=reason_;
                plan.debug.state=StateName(state_); plan.debug.reason=reason_;
                return plan;
            }
            swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0;
            obstacle_votes_.clear(); last_mask_ms_=-1;
        } else if(swerve_!=Swerve::NONE || returning) {
            plan.cmd.cms=std::min({plan.cmd.cms,kSwerveCms,params.max_speed_cms});
            plan.reason=returning ? "VE_PHAI" : swerve_==Swerve::LEFT ? "NE_TRAI" : "NE_QUA";
            plan.debug.reason=plan.reason; plan.debug.speed_cms=plan.cmd.cms;
        }
        sent_cms_=plan.cmd.cms; swerve_ms_=now_ms;
    }
    if(cancelled) plan.reason=plan.debug.reason="HUY_RE";
    if(sensor_cancelled && result.debug.state=="BAM_LAN") plan.reason=plan.debug.reason="HUY_RE_CAM_BIEN";
    if(barrier_fault_stop) { plan.cmd.cms=0; plan.reason=plan.debug.reason="DUNG_RAO_LOI_CAM_BIEN"; plan.debug.speed_cms=0; }
    if(person_hold) { plan.cmd.cms=0; plan.reason=plan.debug.reason="NGUOI_DI_BO"; plan.debug.speed_cms=0; }
    sent_angle_=plan.cmd.deg; command_ms_=now_ms;
    return plan;
}
}
