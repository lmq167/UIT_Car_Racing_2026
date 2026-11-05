#include "control/Planner.h"
#include <algorithm>
#include <cmath>

namespace control {
const char *StateName(PlannerState state) {
    switch(state) {
        case PlannerState::CHO_SAN_SANG: return "CHO_SAN_SANG";
        case PlannerState::BAM_LAN: return "BAM_LAN";
        case PlannerState::DUNG: return "DUNG";
        case PlannerState::CHO_DEN_XUAT_PHAT: return "CHO_DEN_XUAT_PHAT";
        case PlannerState::DUNG_DEN_DO: return "DUNG_DEN_DO";
    }
    return "DUNG";
}
void Planner::Reset(bool preserve_lights) {
    lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; green_run_=0; wait_since_ms_=-1;
    // Epoch đổi phải xoá lịch sử làn, nhưng Space/G/R không xoá pha đèn P2.
    if(preserve_lights) return;
    votes_.clear(); red_ms_=-1; red_x_=0; last_frame_=0; red_latched_=false; start_cleared_=false;
}
Plan Planner::Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params) {
    params.Validate();
    Plan plan; plan.reference=input.reference;
    const bool image_fresh=input.image_ms>=0 && input.image_ms<=now_ms && now_ms-input.image_ms<=kStaleImageMs;
    const bool mask_fresh=!input.mask.empty() && input.mask_ms>=0 && input.mask_ms<=now_ms &&
                          now_ms-input.mask_ms<=params.mask_max_age_ms;
    bool entered_wait=false, released=false;
    if(input.stop) {
        state_=PlannerState::DUNG; reason_="SPACE"; lane_.Reset(); green_run_=0;
    } else if(state_!=PlannerState::CHO_SAN_SANG && state_!=PlannerState::DUNG && !image_fresh) {
        state_=PlannerState::DUNG; reason_=kMissingImageReason; lane_.Reset(); green_run_=0;
    } else if(input.go && (state_==PlannerState::CHO_SAN_SANG || state_==PlannerState::DUNG)) {
        if(!image_fresh || !mask_fresh) {
            plan.state=state_; plan.reason=image_fresh ? "go ignored MAT_NAN_CU" : "go ignored MAT_ANH";
            plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
        }
        state_=!input.yolo_enabled ? PlannerState::BAM_LAN : red_latched_ ? PlannerState::DUNG_DEN_DO :
            (params.start_wait_green && !start_cleared_) ? PlannerState::CHO_DEN_XUAT_PHAT : PlannerState::BAM_LAN;
        reason_=StateName(state_); lane_.Reset(); green_run_=0; wait_since_ms_=now_ms; entered_wait=true;
    }
    bool red=false,green=false,off=false; double red_x=0;
    const auto &batch=input.detections;
    const bool fresh=batch.frame_id && batch.captured_ms>=0 && batch.captured_ms<=now_ms &&
                     now_ms-batch.captured_ms<=params.det_max_age_ms;
    if(input.yolo_enabled) {
        if(!votes_.empty() && now_ms-votes_.back().ms>params.det_max_age_ms) green_run_=0;
        votes_.erase(std::remove_if(votes_.begin(),votes_.end(),[&](const Vote& vote) {
            return vote.ms>now_ms || now_ms-vote.ms>params.det_max_age_ms;
        }),votes_.end());
        if(fresh) for(const auto &d:batch.items) {
            double conf=0,area=0;
            if(d.class_name=="red") { conf=params.red_conf_min; area=params.red_box_min_area_frac; }
            else if(d.class_name=="green") { conf=params.green_conf_min; area=params.green_box_min_area_frac; }
            else if(d.class_name=="off") { conf=params.off_conf_min; area=params.off_box_min_area_frac; }
            else continue;
            const auto &b=d.box;
            if(d.captured_ms!=batch.captured_ms || !std::isfinite(d.confidence) || d.confidence<conf || d.confidence>1 ||
               !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
               b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720 ||
               double(b.width)*b.height/(1280.0*720.0)<area) continue;
            if(d.class_name=="red") { red=true; red_x=std::max(red_x,(double(b.x)+b.width)/1280.0); }
            else if(d.class_name=="green") green=true;
            else off=true;
        }
        if(!fresh || red || off || !green) green_run_=0;
        if(batch.frame_id>last_frame_) {
            last_frame_=batch.frame_id;
            if(fresh) {
                if(red) { red_ms_=batch.captured_ms; red_x_=red_x; }
                votes_.push_back({batch.captured_ms,red,green && !red && !off});
                if(!entered_wait && !input.stop && image_fresh && batch.captured_ms>wait_since_ms_ && green && !red && !off &&
                   (state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO)) ++green_run_;
            }
        }
        while(votes_.size()>size_t(params.vote_window)) votes_.pop_front();
        int reds=0,greens=0;
        for(const auto &v:votes_) { reds+=v.red; greens+=v.green; }
        const bool red_fresh=red_ms_>=0 && red_ms_<=now_ms && now_ms-red_ms_<=params.det_max_age_ms;
        if(red_fresh && reds>=params.vote_min && red_x_>=params.red_stop_x_frac) {
            if(!red_latched_) { green_run_=0; wait_since_ms_=now_ms; }
            red_latched_=true;
        }
        if(!input.stop && state_!=PlannerState::DUNG && state_!=PlannerState::CHO_SAN_SANG) {
            if(red_latched_) { state_=PlannerState::DUNG_DEN_DO; reason_="DUNG_DEN_DO"; }
            if((state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO) &&
               fresh && !red && !off && green && greens>=params.vote_min && reds<params.vote_min && green_run_>=params.green_frames) {
                red_latched_=false; start_cleared_=true; green_run_=0;
                votes_.clear(); red_ms_=-1; red_x_=0;
                state_=PlannerState::BAM_LAN; lane_.Reset(); released=true;
            }
        }
        if(released) reason_="DEN_XANH";
        else if(state_==PlannerState::BAM_LAN && red_fresh && reds>=params.vote_min) reason_="TIEP_CAN_DEN_DO";
        else if(state_==PlannerState::BAM_LAN) reason_="BAM_LAN";
    }
    plan.state=state_;
    if(state_!=PlannerState::BAM_LAN) {
        plan.reason=reason_; plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
    }
    const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,input.reference);
    plan.cmd=result.cmd; plan.debug=result.debug; plan.reason=result.debug.reason;
    if(input.yolo_enabled && (!fresh || reason_=="TIEP_CAN_DEN_DO" || released)) {
        if(!fresh || reason_=="TIEP_CAN_DEN_DO") plan.cmd.cms=std::min({plan.cmd.cms,params.approach_cms,params.max_speed_cms});
        if(result.debug.state=="BAM_LAN") {
            plan.reason=released ? "DEN_XANH" : !fresh ? "MAT_YOLO" : reason_; plan.debug.reason=plan.reason;
        }
        plan.debug.speed_cms=plan.cmd.cms;
    }
    return plan;
}
}
