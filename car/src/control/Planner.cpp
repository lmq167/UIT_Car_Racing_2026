#include "control/Planner.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {
constexpr double kBarrierConfMin=0.5; // Hằng tin cậy rào/vòng xuyến như vật YOLO khác.
constexpr double kPersonConfMin=0.5; // Hằng như rào; tên lớp người chờ model mới.
constexpr int64_t kSensorGlitchMs=250;  // Mẫu cảm biến hỏng ngắn hơn mức này thì dừng giữ góc, dài hơn thì hủy rẽ (Quân duyệt 07/11).
constexpr double kSignMinWidthFrac=45.0/1280;  // TẠM: biển hẹp hơn (theo bề rộng ảnh) là biển xa/chặng khác, không tính phiếu; biển giả đo được rộng <= 36 px.
// Góc lái rẽ theo hình học sa bàn (làn 30 cm, bo R=50): rẽ phải/vào 1 làn cần bán kính đường xe ~65 cm (lái nhiều hơn ~12%),
// rẽ trái vào đường 2 làn (rào) ~95 cm (lái ít hơn ~20%); biển/vòng xuyến rẽ trái chưa biết số làn nên giữ turn_steer_deg.
// Rào work 1 (vào đường 1 làn) lệch giữa (tâm hộp 0,43-0,54), work 2 (vào đường 2 làn) lệch phải (0,70-0,71): đo ở hai phiên 07/11.
constexpr double kWideBarrierCx=0.62, kWideBarrierMaxCx=0.82, kNarrowBarrierEarly=0.05;
// Rẽ phải thoát sớm hơn một chút: log 4 lần rẽ phải xong ở -80 độ rồi bộ bám làn phải trả lại 5-8 độ.
constexpr double kRightTurnYawFactor=0.92;
constexpr double kReturnMaxError=0.5, kReturnAlignedError=0.12, kReturnCapFactor=5;
double TurnSteer(const ControlParams &p, int direction, bool barrier_turn, bool wide_barrier) {
    const double s=direction<0 ? p.turn_steer_deg*1.12 : barrier_turn ? p.turn_steer_deg*(wide_barrier ? 0.8 : 1.12) : p.turn_steer_deg;
    return std::min(s,p.steer_max_deg);
}
// Sai số góc gói về [-180, 180).
double WrapDeg(double d) { d=std::fmod(d+180.0,360.0); if(d<0) d+=360.0; return d-180.0; }
double HoleFraction(const cv::Mat &mask, double *centroid_col_frac) {
    *centroid_col_frac=std::nan("");
    if(mask.empty() || mask.type()!=CV_8UC1) return 0;
    int64_t holes=0, total=0, columns=0;
    for(int y=int(0.40*mask.rows); y<int(0.95*mask.rows); ++y) {
        const auto *row=mask.ptr<unsigned char>(y);
        int first=0,last=mask.cols-1;
        while(first<mask.cols && row[first]==0) ++first;
        while(last>first && row[last]==0) --last;
        if(first>=last) continue;
        total+=last-first+1;
        for(int x=first; x<=last; ++x) if(row[x]==0) { ++holes; columns+=x; }
    }
    if(holes) *centroid_col_frac=double(columns)/holes/mask.cols;
    return total ? double(holes)/total : 0;
}
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
    }
    return "DUNG";
}
void Planner::ClearTurn() {
    turn_direction_=0; turning_=false; barrier_turn_=false; barrier_wide_turn_=false;
    arm_odo_=turn_odo_=yaw0_=progress_odo_=0; turn_ms_=progress_ms_=-1;
    for(auto &vote:votes_) vote.left=vote.right=false;
}
void Planner::Reset(bool preserve_lights) {
    ClearTurn(); sent_angle_=0; command_ms_=-1; sensor_bad_since_=-1; sensor_hold_=false;
    lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; green_run_=0; wait_since_ms_=-1;
    obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
    swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; approach_=false; person_stop_=false; return_cm_=-1;
    rb_hold_spent_=true; rb_holding_=false;  // sau Space/G/R không giữ hướng bằng yaw0 cũ
    sensor_good_ms_=-1;
    // Epoch đổi phải xoá lịch sử làn, nhưng Space/G/R không xoá pha đèn P2.
    if(preserve_lights) return;
    votes_.clear(); red_ms_=-1; red_x_=0; last_frame_=0; red_latched_=false; start_cleared_=false; barrier_done_=false;
}
Plan Planner::Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params) {
    params.Validate();
    Plan plan; plan.reference=input.reference;
    const bool image_fresh=input.image_ms>=0 && input.image_ms<=now_ms && now_ms-input.image_ms<=kStaleImageMs;
    const bool mask_fresh=!input.mask.empty() && input.mask_ms>=0 && input.mask_ms<=now_ms &&
                          now_ms-input.mask_ms<=params.mask_max_age_ms;
    const bool sensor_bad=!input.sensor_ok || !std::isfinite(input.yaw_deg) || !std::isfinite(input.odo_cm) ||
        input.sensor_ms<0 || input.sensor_ms>now_ms || now_ms-input.sensor_ms>200;
    bool entered_wait=false, released=false, barrier_slow=false, barrier_fault_stop=false, sign_batch=false;
    if(input.stop) {
        ClearTurn(); sent_angle_=0; command_ms_=-1; rb_hold_spent_=true; rb_holding_=false;
        state_=PlannerState::DUNG; reason_="SPACE"; lane_.Reset(); green_run_=0;
    } else if(state_!=PlannerState::CHO_SAN_SANG && state_!=PlannerState::DUNG && !image_fresh) {
        ClearTurn();
        state_=PlannerState::DUNG; reason_=kMissingImageReason; lane_.Reset(); green_run_=0;
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
    }
    bool red=false,green=false,off=false,left=false,right=false,barrier_near=false,barrier_wide=false,barrier_seen=false,round=false,people=false; double red_x=0;
    bool cancelled=false, sensor_cancelled=false;
    auto fault=[&]() {
        sensor_cancelled=turning_;
        if(turning_) { state_=PlannerState::BAM_LAN; lane_.Reset(); }
        if(barrier_turn_) barrier_done_=false;  // rẽ qua rào bị hủy: rào còn gần thì dừng (Q7), hồi lỗi thì rẽ lại
        ClearTurn(); turn_fault_=true; rb_pending_=0;  // không để rẽ vòng xuyến chờ bắn muộn sau khi hồi
    };
    // Lỗi cảm biến/rẽ chỉ tạm: cảm biến tươi liên tục >= 1 s thì cho rẽ lại (IMU lỗi lúc khởi động luôn sensor_ok sai nên không hồi).
    if(params.turn_enable && turn_fault_) {
        if(sensor_bad) sensor_good_ms_=-1;
        else if(sensor_good_ms_<0) sensor_good_ms_=now_ms;
        else if(now_ms-sensor_good_ms_>=1000) { turn_fault_=false; sensor_good_ms_=-1; }
    }
    if(sensor_bad) { if(sensor_bad_since_<0) sensor_bad_since_=now_ms; } else sensor_bad_since_=-1;
    sensor_hold_=false;
    if(params.turn_enable && !turn_fault_) {
        if(sensor_bad) {
            // Mẫu hỏng thoáng qua: giữ nguyên lượt rẽ nhưng xe dừng (không biết chuyển động); hỏng từ kSensorGlitchMs thì hủy.
            if(now_ms-sensor_bad_since_<kSensorGlitchMs) sensor_hold_=true; else fault();
        } else if(turning_) {
            // Đang dừng vì người đi bộ: không tính thời gian đứng vào hạn rẽ/kẹt.
            if(person_stop_ && command_ms_>=0 && now_ms>command_ms_) { turn_ms_+=now_ms-command_ms_; progress_ms_=now_ms; }
            if(std::abs(input.odo_cm-turn_odo_)>params.turn_max_cm) fault();
            else if(now_ms-turn_ms_>params.turn_max_ms) fault();
            else if(std::abs(input.odo_cm-progress_odo_)>=1) { progress_odo_=input.odo_cm; progress_ms_=now_ms; }
            else if(std::abs(input.yaw_deg-yaw0_)<params.turn_yaw_deg && now_ms-progress_ms_>=1500) fault();
        }
    }
    const auto &batch=input.detections;
    const bool fresh=batch.frame_id && batch.captured_ms>=0 && batch.captured_ms<=now_ms &&
                     now_ms-batch.captured_ms<=params.det_max_age_ms;
    person_stop_=false;
    if(input.yolo_enabled) {
        if(!votes_.empty() && now_ms-votes_.back().ms>params.det_max_age_ms) green_run_=0;
        votes_.erase(std::remove_if(votes_.begin(),votes_.end(),[&](const Vote& vote) {
            return vote.ms>now_ms || now_ms-vote.ms>params.det_max_age_ms;
        }),votes_.end());
        if(fresh) for(const auto &d:batch.items) {
            const bool signLeft=d.class_name=="left" || d.class_name=="no_right";
            const bool signRight=d.class_name=="right" || d.class_name=="no_left";
            const bool barrier=params.work_barrier_action && d.class_name=="work_barrier";
            const bool roundabout=d.class_name=="round_barrier";
            const bool person=d.class_name=="people" || d.class_name=="person";
            double conf=0,area=0;
            if(d.class_name=="red") { conf=params.red_conf_min; area=params.red_box_min_area_frac; }
            else if(d.class_name=="green") { conf=params.green_conf_min; area=params.green_box_min_area_frac; }
            else if(d.class_name=="off") { conf=params.off_conf_min; area=params.off_box_min_area_frac; }
            else if(barrier || roundabout) conf=kBarrierConfMin;
            else if(person) conf=kPersonConfMin;
            else if(!(params.turn_enable && !turning_ && !turn_fault_ && (signLeft || signRight))) continue;  // đã chốt hướng vẫn theo dõi biển để biết khi nào nó rời khung
            const auto &b=d.box;
            if(d.captured_ms!=batch.captured_ms || !std::isfinite(d.confidence) || d.confidence<conf || d.confidence>1 ||
               !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
               b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720 ||
               double(b.width)*b.height/(1280.0*720.0)<area || ((signLeft || signRight) && b.width<kSignMinWidthFrac*1280)) continue;
            if(d.class_name=="red") { red=true; red_x=std::max(red_x,(double(b.x)+b.width)/1280.0); }
            else if(d.class_name=="green") green=true;
            else if(d.class_name=="off") off=true;
            else if(roundabout) round=true;
            else if(person) people=true;
            else if(barrier) {
                // Đáy hộp gần xe; tâm ngoài 0,25-0,75 bề rộng là rào bên đường, không chắn trước (log 1793955267).
                const double bottom=(double(b.y)+b.height)/720.0, center=(double(b.x)+b.width/2)/1280.0;
                barrier_seen=barrier_seen || bottom>=params.work_barrier_y2_frac-0.10;
                // Work 2 (hộp lệch phải) vào đường 2 làn: dải tâm tới 0,82; work 1 (hộp giữa) kích hoạt sớm hơn 0,05 theo đáy hộp.
                const bool wide=center>=kWideBarrierCx;
                const bool near_wide=wide && bottom>=params.work_barrier_y2_frac && center<=kWideBarrierMaxCx;
                const bool near_narrow=!wide && center>=0.25 && bottom>=params.work_barrier_y2_frac-kNarrowBarrierEarly;
                barrier_near=barrier_near || near_wide || near_narrow;
                barrier_wide=barrier_wide || near_wide;
            }
            else { left=left || signLeft; right=right || signRight; }
        }
        if(!fresh || red || off || !green) green_run_=0;
        if(batch.frame_id>last_frame_) {
            last_frame_=batch.frame_id;
            sign_batch=fresh && (left || right);  // lô mới có biển hợp lệ (đủ rộng)
            if(fresh) {
                if(red) { red_ms_=batch.captured_ms; red_x_=red_x; }
                votes_.push_back({batch.captured_ms,red,green && !red && !off,left,right,barrier_near,barrier_seen,round,people,barrier_wide});
                if(!entered_wait && !input.stop && image_fresh && batch.captured_ms>wait_since_ms_ && green && !red && !off &&
                   (state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO)) ++green_run_;
            }
        }
        while(votes_.size()>size_t(params.vote_window)) votes_.pop_front();
        int reds=0,greens=0,lefts=0,rights=0,barriers=0,barriers_wide=0,barriers_seen=0,rounds=0,persons=0;
        for(const auto &v:votes_) {
            reds+=v.red; greens+=v.green; lefts+=v.left; rights+=v.right; barriers+=v.barrier_near; barriers_wide+=v.barrier_wide; barriers_seen+=v.barrier_seen;
            rounds+=v.round; persons+=v.person;
        }
        person_stop_=persons>=params.vote_min;
        // Lần qua vòng xuyến; quãng theo encoder (odo NaN khi encoder lỗi thì không kết thúc lần qua).
        const bool rb_present=rounds>=params.vote_min;
        if(rb_present) {
            if(!rb_in_pass_) { rb_in_pass_=true; rb_yaw0_=input.yaw_deg; rb_hold_odo_=input.odo_cm; rb_hold_spent_=false; }
            rb_losing_=false;
        } else if(rb_in_pass_) {
            if(!rb_losing_) { rb_losing_=true; rb_lost_odo_=input.odo_cm; }
            else if(std::abs(input.odo_cm-rb_lost_odo_)>=params.rb_lost_cm) {
                rb_in_pass_=rb_losing_=false; ++rb_passes_; rb_after_odo_=input.odo_cm;
                rb_pending_=rb_passes_==1 ? 1 : rb_passes_==2 ? -1 : 0;  // lần 1 rẽ trái, lần 2 rẽ phải, sau đó không áp
            }
        }
        barrier_slow=barriers_seen>=params.vote_min;
        if(!barriers_seen) barrier_done_=false;
        // Rào gần đủ phiếu: rẽ trái ngay bằng máy rẽ IMU (vòng kín theo yaw), chốt quyết định tới hết rẽ.
        if(params.work_barrier_action && params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && !barrier_done_ && !rb_holding_ && !rb_present &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && barriers>=params.vote_min) {
            turn_direction_=1; arm_odo_=input.odo_cm; barrier_done_=true; barrier_turn_=true; barrier_wide_turn_=barriers_wide*2>barriers;
            turning_=true; yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm;
            turn_ms_=progress_ms_=now_ms; state_=PlannerState::RE_TRAI;
        }
        // Rào gần mà không rẽ được (lỗi cảm biến tạm hoặc IMU lỗi từ khởi động): dừng trước rào, không đi thẳng.
        // Lỗi tạm hồi (1 s tươi) thì luật trên rẽ tiếp; G sau Space bỏ qua rào này (barrier_done_).
        barrier_fault_stop=params.work_barrier_action && params.turn_enable && (turn_fault_ || sensor_bad) && !barrier_done_ && !rb_present &&
            state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && barriers>=params.vote_min;
        // Ngã T không biển sau vòng xuyến: đi thêm rb_turn1/2_cm rồi rẽ; còn phiếu round_barrier thì không rẽ.
        if(rb_pending_ && params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && !rb_present && !rb_holding_ &&
           state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE &&
           std::abs(input.odo_cm-rb_after_odo_)>=(rb_pending_==1 ? params.rb_turn1_cm : params.rb_turn2_cm)) {
            turn_direction_=rb_pending_; arm_odo_=input.odo_cm; rb_pending_=0;
            turning_=true; yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm;
            turn_ms_=progress_ms_=now_ms; state_=turn_direction_==1 ? PlannerState::RE_TRAI : PlannerState::RE_PHAI;
        }
        if(params.turn_enable && !turn_fault_ && !sensor_bad && !turn_direction_ && state_==PlannerState::BAM_LAN &&
           (lefts>=params.vote_min)!=(rights>=params.vote_min)) {  // mẫu hỏng: không chốt mốc quãng NaN
            turn_direction_=lefts>=params.vote_min ? 1 : -1; arm_odo_=input.odo_cm; rb_pending_=0;
        } else if(params.turn_enable && !turn_fault_ && turn_direction_ && !turning_ && !sensor_bad && sign_batch &&
                  (turn_direction_==1 ? left : right)) {
            arm_odo_=input.odo_cm;  // biển còn trong ảnh: mốc rẽ dời theo, chỉ đếm turn_start_cm kể từ lúc biển rời khung hình
        }
        const bool red_fresh=red_ms_>=0 && red_ms_<=now_ms && now_ms-red_ms_<=params.det_max_age_ms;
        if(red_fresh && reds>=params.vote_min && red_x_>=params.red_stop_x_frac) {
            if(!red_latched_) { green_run_=0; wait_since_ms_=now_ms; }
            red_latched_=true;
        }
        if(!input.stop && state_!=PlannerState::DUNG && state_!=PlannerState::CHO_SAN_SANG) {
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
        swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; return_cm_=-1;
        if(params.obstacle_action==2) {
            obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
        }
    }
    // shortcut: quãng theo lệnh gửi, cần encoder khi có phép đo quãng đường thực.
    if(swerve_==Swerve::PASS && swerve_ms_>=0 && now_ms>swerve_ms_)
        pass_cm_+=sent_cms_*(now_ms-swerve_ms_)/1000.0;
    // Pha về làn phải (VE_PHAI) dài bằng quãng né obstacle_pass_cm, kẹp tốc độ như pha né.
    if(return_cm_>=0 && swerve_ms_>=0 && now_ms>swerve_ms_) return_cm_+=sent_cms_*(now_ms-swerve_ms_)/1000.0;
    // Về làn phải: tối thiểu obstacle_pass_cm; với pair_mode=1 chờ tới khi về tới làn (sai số nhỏ) hoặc đủ kReturnCapFactor lần quãng đó.
    if(return_cm_>=params.obstacle_pass_cm &&
       (params.pair_mode!=1 || std::abs(return_e_)<kReturnAlignedError || return_cm_>=kReturnCapFactor*params.obstacle_pass_cm)) return_cm_=-1;
    if(params.obstacle_action!=0 &&
       (state_==PlannerState::BAM_LAN || state_==PlannerState::DUNG_VAT_CAN)) {
        if(mask_fresh && input.mask_ms!=last_mask_ms_) {
            last_mask_ms_=input.mask_ms;
            double centroid=0;
            const double fraction=HoleFraction(input.mask,&centroid);
            const bool obstacle=fraction>=params.obstacle_hole_frac && centroid>=0.35 && centroid<=0.85;
            obstacle_votes_.push_back(obstacle);
            while(obstacle_votes_.size()>size_t(params.vote_window)) obstacle_votes_.pop_front();
            // Đang qua vòng xuyến: lòng vòng là "hố" trong mặt nạ nhưng phải đi thẳng, không né (ảnh 07/11 phiên 9445, 211-219 s).
            if(state_==PlannerState::BAM_LAN && !rb_holding_ && !(params.obstacle_action==2 && rb_in_pass_) &&
               std::count(obstacle_votes_.begin(),obstacle_votes_.end(),true)>=params.vote_min) {
                if(params.obstacle_action==1) {
                    state_=PlannerState::DUNG_VAT_CAN; reason_="VAT_CAN"; obstacle_clear_run_=0;
                } else if(swerve_!=Swerve::LEFT) {
                    swerve_=Swerve::LEFT; pass_cm_=0; obstacle_clear_run_=0;
                }
            }
            if(params.obstacle_action==2 && swerve_==Swerve::LEFT) {
                obstacle_clear_run_=fraction>=params.obstacle_hole_frac*0.5 ? 0 : obstacle_clear_run_+1;
                if(obstacle_clear_run_>=params.vote_window) {
                    swerve_=Swerve::PASS; pass_cm_=0; obstacle_votes_.clear();
                }
            } else if(state_==PlannerState::DUNG_VAT_CAN) {
                obstacle_clear_run_=fraction>=params.obstacle_hole_frac*0.5 ? 0 : obstacle_clear_run_+1;
                if(obstacle_clear_run_>=params.vote_window) {
                    state_=PlannerState::BAM_LAN; reason_="BAM_LAN"; lane_.Reset();
                    obstacle_votes_.clear(); obstacle_clear_run_=0;
                }
            }
        }
    } else {
        obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
    }
    if(swerve_==Swerve::PASS && pass_cm_>=params.obstacle_pass_cm) {
        swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0; obstacle_votes_.clear();
        lane_.Reset(true); return_cm_=0; return_e_=1;
    }
    const bool returning=return_cm_>=0;
    if(params.turn_enable && turn_direction_ && state_!=PlannerState::DUNG && state_!=PlannerState::DUNG_DEN_DO &&
       state_!=PlannerState::DUNG_VAT_CAN) {
        if(!mask_fresh) {
            if(barrier_turn_) barrier_done_=false;  // rẽ rào bị hủy vì mặt nạ cũ: rào còn gần thì xét lại (rẽ hoặc dừng Q7)
            ClearTurn(); state_=PlannerState::BAM_LAN;
        } else if(turning_ && !sensor_bad && std::abs(input.yaw_deg-yaw0_)>=params.turn_yaw_deg*(turn_direction_<0 ? kRightTurnYawFactor : 1.0)) {  // mẫu hỏng (kể cả ±inf) không được coi là đã quay đủ
            ClearTurn(); state_=PlannerState::BAM_LAN; lane_.Reset();
        } else if(!turning_) {
            if(!sensor_bad && std::abs(input.odo_cm-arm_odo_)>params.turn_arm_cm) { ClearTurn(); cancelled=true; }
            // Bắt đầu rẽ theo quãng encoder kể từ lúc biển đủ phiếu; đang né/về làn thì chờ bám làn thường.
            else if(state_==PlannerState::BAM_LAN && swerve_==Swerve::NONE && !returning && !rb_holding_ && !sensor_bad &&
                    !(rb_in_pass_ && !rb_losing_) &&  // còn đủ phiếu vòng xuyến thì chưa rẽ (đảo phía trước)
                    std::abs(input.odo_cm-arm_odo_)>=params.turn_start_cm) {
                turning_=true; yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm;
                turn_ms_=progress_ms_=now_ms;
                state_=turn_direction_==1 ? PlannerState::RE_TRAI : PlannerState::RE_PHAI;
            }
        }
    }
    plan.state=state_;
    if(params.turn_enable && turning_ && (state_==PlannerState::RE_TRAI || state_==PlannerState::RE_PHAI)) {
        const double step=sensor_hold_ ? 0 : params.steer_rate_dps*(command_ms_<0 ? 0 : std::max<int64_t>(0,now_ms-command_ms_)/1000.0);  // giữ nguyên góc khi mẫu hỏng
        const double target=turn_direction_*TurnSteer(params,turn_direction_,barrier_turn_,barrier_wide_turn_);
        sent_angle_=std::max(-params.steer_max_deg,std::min(params.steer_max_deg,
            std::max(sent_angle_-step,std::min(sent_angle_+step,target))));
        command_ms_=now_ms;
        plan.cmd={person_stop_ || sensor_hold_ ? 0.0 : params.min_cms,sent_angle_};
        plan.reason=person_stop_ ? "NGUOI_DI_BO" : sensor_hold_ ? "CAM_BIEN_CHO" : StateName(state_);
        plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
        plan.debug.angle_deg=sent_angle_; plan.debug.speed_cms=plan.cmd.cms;
        return plan;
    }
    if(state_!=PlannerState::BAM_LAN) {
        sent_angle_=0; command_ms_=now_ms;
        plan.reason=reason_; plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
    }
    if(swerve_!=Swerve::NONE) plan.reference=LaneRef::LEFT;
    lane_.SetSwerve(swerve_!=Swerve::NONE ? 1 : returning ? -1 : 0);  // pair_mode=1: theo vạch phân làn khi chuyển làn
    const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,plan.reference);
    if(returning) return_e_=result.debug.e;
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
    if(input.yolo_enabled && (!fresh || reason_=="TIEP_CAN_DEN_DO" || released)) {
        if(!fresh || reason_=="TIEP_CAN_DEN_DO") plan.cmd.cms=std::min({plan.cmd.cms,params.approach_cms,params.max_speed_cms});
        if(result.debug.state=="BAM_LAN") {
            plan.reason=released ? "DEN_XANH" : !fresh ? "MAT_YOLO" : reason_; plan.debug.reason=plan.reason;
        }
        plan.debug.speed_cms=plan.cmd.cms;
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
                swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1; return_cm_=-1;
                obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
                plan=Plan{}; plan.reference=input.reference; plan.state=state_; plan.reason=reason_;
                plan.debug.state=StateName(state_); plan.debug.reason=reason_;
                return plan;
            }
            swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0;
            obstacle_votes_.clear(); last_mask_ms_=-1; plan.reference=input.reference;
        } else if(swerve_!=Swerve::NONE || returning) {
            plan.cmd.cms=std::min(plan.cmd.cms,params.approach_cms);
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
