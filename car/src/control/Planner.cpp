#include "control/Planner.h"
#include <algorithm>
#include <cmath>

namespace control {
namespace {
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
    }
    return "DUNG";
}
void Planner::Reset(bool preserve_lights) {
    lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; green_run_=0; wait_since_ms_=-1;
    obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
    swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1;
    // Epoch đổi phải xoá lịch sử làn, nhưng Space/G/R không xoá pha đèn P2.
    if(preserve_lights) {
        for(auto &vote:votes_) vote.barrier_near=false;
        return;
    }
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
    bool red=false,green=false,off=false,barrier_near=false,barrier_ready=false; double red_x=0;
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
            else if(d.class_name=="work_barrier") conf=0.5;
            else continue;
            const auto &b=d.box;
            if(d.captured_ms!=batch.captured_ms || !std::isfinite(d.confidence) || d.confidence<conf || d.confidence>1 ||
               !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
               b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720 ||
               double(b.width)*b.height/(1280.0*720.0)<area) continue;
            if(d.class_name=="red") { red=true; red_x=std::max(red_x,(double(b.x)+b.width)/1280.0); }
            else if(d.class_name=="green") green=true;
            else if(d.class_name=="off") off=true;
            else if(double(b.y)+b.height>=0.70*720) barrier_near=true;
        }
        if(!fresh || red || off || !green) green_run_=0;
        if(batch.frame_id>last_frame_) {
            last_frame_=batch.frame_id;
            if(fresh) {
                if(red) { red_ms_=batch.captured_ms; red_x_=red_x; }
                votes_.push_back({batch.captured_ms,red,green && !red && !off,barrier_near});
                if(!entered_wait && !input.stop && image_fresh && batch.captured_ms>wait_since_ms_ && green && !red && !off &&
                   (state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO)) ++green_run_;
            }
        }
        while(votes_.size()>size_t(params.vote_window)) votes_.pop_front();
        int reds=0,greens=0,barriers=0;
        for(const auto &v:votes_) { reds+=v.red; greens+=v.green; barriers+=v.barrier_near; }
        barrier_ready=params.work_barrier_action==1 && barriers>=params.vote_min;
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
    bool returning=false;
    if((params.obstacle_action!=2 && params.work_barrier_action==0) || state_!=PlannerState::BAM_LAN || entered_wait) {
        swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1;
        if(params.obstacle_action==2 || (params.work_barrier_action==1 && state_!=PlannerState::DUNG_VAT_CAN)) {
            obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
        }
    }
    if(barrier_ready && state_==PlannerState::BAM_LAN && swerve_!=Swerve::LEFT) {
        swerve_=Swerve::LEFT; pass_cm_=0; obstacle_clear_run_=0;
    }
    // shortcut: quãng theo lệnh gửi, cần encoder khi có phép đo quãng đường thực.
    if(swerve_==Swerve::PASS && swerve_ms_>=0 && now_ms>swerve_ms_)
        pass_cm_+=sent_cms_*(now_ms-swerve_ms_)/1000.0;
    if(params.obstacle_action!=0 &&
       (state_==PlannerState::BAM_LAN || state_==PlannerState::DUNG_VAT_CAN)) {
        if(mask_fresh && input.mask_ms!=last_mask_ms_) {
            last_mask_ms_=input.mask_ms;
            double centroid=0;
            const double fraction=HoleFraction(input.mask,&centroid);
            const bool obstacle=fraction>=params.obstacle_hole_frac && centroid>=0.35 && centroid<=0.85;
            obstacle_votes_.push_back(obstacle);
            while(obstacle_votes_.size()>size_t(params.vote_window)) obstacle_votes_.pop_front();
            if(state_==PlannerState::BAM_LAN &&
               std::count(obstacle_votes_.begin(),obstacle_votes_.end(),true)>=params.vote_min) {
                if(params.obstacle_action==1 && swerve_==Swerve::NONE) {
                    state_=PlannerState::DUNG_VAT_CAN; reason_="VAT_CAN"; obstacle_clear_run_=0;
                } else if(swerve_!=Swerve::LEFT) {
                    swerve_=Swerve::LEFT; pass_cm_=0; obstacle_clear_run_=0;
                }
            }
            if(swerve_==Swerve::LEFT) {
                obstacle_clear_run_=fraction>=params.obstacle_hole_frac*0.5 ? 0 : obstacle_clear_run_+1;
                if(!barrier_ready && obstacle_clear_run_>=params.vote_window) {
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
    if(params.obstacle_action==0 && swerve_==Swerve::LEFT && !barrier_ready) {
        swerve_=Swerve::PASS; pass_cm_=0;
    }
    if(swerve_==Swerve::PASS && pass_cm_>=params.obstacle_pass_cm) {
        swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0; obstacle_votes_.clear();
        lane_.Reset(); returning=true;
    }
    plan.state=state_;
    if(state_!=PlannerState::BAM_LAN) {
        plan.reason=reason_; plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
    }
    if(swerve_!=Swerve::NONE) plan.reference=LaneRef::LEFT;
    const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,plan.reference);
    plan.cmd=result.cmd; plan.debug=result.debug; plan.reason=result.debug.reason;
    if(input.yolo_enabled && (!fresh || reason_=="TIEP_CAN_DEN_DO" || released)) {
        if(!fresh || reason_=="TIEP_CAN_DEN_DO") plan.cmd.cms=std::min({plan.cmd.cms,params.approach_cms,params.max_speed_cms});
        if(result.debug.state=="BAM_LAN") {
            plan.reason=released ? "DEN_XANH" : !fresh ? "MAT_YOLO" : reason_; plan.debug.reason=plan.reason;
        }
        plan.debug.speed_cms=plan.cmd.cms;
    }
    if(params.obstacle_action==2 || params.work_barrier_action==1) {
        if(result.debug.state!="BAM_LAN") {
            if(swerve_!=Swerve::NONE) {
                state_=PlannerState::DUNG; reason_="MAT_LAN_NE"; lane_.Reset(); green_run_=0;
                swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1;
                obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
                plan=Plan{}; plan.reference=input.reference; plan.state=state_; plan.reason=reason_;
                plan.debug.state=StateName(state_); plan.debug.reason=reason_;
                return plan;
            }
            swerve_=Swerve::NONE; pass_cm_=0; obstacle_clear_run_=0;
            obstacle_votes_.clear(); last_mask_ms_=-1; plan.reference=input.reference;
        } else if(swerve_!=Swerve::NONE || returning) {
            if(!returning) plan.cmd.cms=std::min(plan.cmd.cms,params.approach_cms);
            plan.reason=returning ? "VE_PHAI" : swerve_==Swerve::LEFT ? "NE_TRAI" : "NE_QUA";
            plan.debug.reason=plan.reason; plan.debug.speed_cms=plan.cmd.cms;
        }
        sent_cms_=plan.cmd.cms; swerve_ms_=now_ms;
    }
    return plan;
}
}
