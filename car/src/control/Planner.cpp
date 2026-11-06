#include "control/Planner.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

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
        case PlannerState::RE_TRAI: return "RE_TRAI";
        case PlannerState::RE_PHAI: return "RE_PHAI";
    }
    return "DUNG";
}
void Planner::ClearTurn() {
    turn_direction_=0; edge_run_=0; turning_=false; edge_mask_ms_=-1;
    arm_odo_=turn_odo_=yaw0_=progress_odo_=0; turn_ms_=progress_ms_=-1;
    for(auto &vote:votes_) vote.left=vote.right=false;
}
void Planner::Reset(bool preserve_lights) {
    ClearTurn(); turn_fault_=false; sent_angle_=0; command_ms_=-1;
    lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; green_run_=0; wait_since_ms_=-1;
    obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
    swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1;
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
        ClearTurn(); turn_fault_=false; sent_angle_=0; command_ms_=-1;
        state_=PlannerState::DUNG; reason_="SPACE"; lane_.Reset(); green_run_=0;
    } else if(state_!=PlannerState::CHO_SAN_SANG && state_!=PlannerState::DUNG && !image_fresh) {
        ClearTurn();
        state_=PlannerState::DUNG; reason_=kMissingImageReason; lane_.Reset(); green_run_=0;
    } else if(input.go && !turn_fault_ && (state_==PlannerState::CHO_SAN_SANG || state_==PlannerState::DUNG)) {
        if(!image_fresh || !mask_fresh) {
            plan.state=state_; plan.reason=image_fresh ? "go ignored MAT_NAN_CU" : "go ignored MAT_ANH";
            plan.debug.state=StateName(state_); plan.debug.reason=plan.reason; return plan;
        }
        state_=!input.yolo_enabled ? PlannerState::BAM_LAN : red_latched_ ? PlannerState::DUNG_DEN_DO :
            (params.start_wait_green && !start_cleared_) ? PlannerState::CHO_DEN_XUAT_PHAT : PlannerState::BAM_LAN;
        reason_=StateName(state_); lane_.Reset(); green_run_=0; wait_since_ms_=now_ms; entered_wait=true;
    }
    bool red=false,green=false,off=false,left=false,right=false; double red_x=0;
    bool cancelled=false;
    auto sensorReason=[&]() -> const char* {
        if(!input.sensor_ok || !std::isfinite(input.yaw_deg) || !std::isfinite(input.odo_cm)) return "LOI_CAM_BIEN";
        if(input.sensor_ms<0 || input.sensor_ms>now_ms || now_ms-input.sensor_ms>200) return "CAM_BIEN_CU";
        return nullptr;
    };
    auto fault=[&](const char* reason) {
        ClearTurn(); turn_fault_=true; state_=PlannerState::DUNG; reason_=reason;
        lane_.Reset(); sent_angle_=0;
    };
    // Lỗi phiên rẽ phải thắng đèn đỏ và không được G tự mở lại.
    if(params.turn_enable && turn_direction_ && !input.stop && state_!=PlannerState::DUNG) {
        if(const char* error=sensorReason()) fault(error);
        else if(turning_ && std::abs(input.yaw_deg-yaw0_)<params.turn_yaw_deg) {
            if(std::abs(input.odo_cm-turn_odo_)>params.turn_max_cm) fault("RE_QUA_CM");
            else if(now_ms-turn_ms_>params.turn_max_ms) fault("RE_QUA_MS");
            else if(std::abs(input.odo_cm-progress_odo_)>=1) { progress_odo_=input.odo_cm; progress_ms_=now_ms; }
            else if(now_ms-progress_ms_>=1500) fault("KET_RE");
        }
    }
    const auto &batch=input.detections;
    const bool fresh=batch.frame_id && batch.captured_ms>=0 && batch.captured_ms<=now_ms &&
                     now_ms-batch.captured_ms<=params.det_max_age_ms;
    if(input.yolo_enabled) {
        if(!votes_.empty() && now_ms-votes_.back().ms>params.det_max_age_ms) green_run_=0;
        votes_.erase(std::remove_if(votes_.begin(),votes_.end(),[&](const Vote& vote) {
            return vote.ms>now_ms || now_ms-vote.ms>params.det_max_age_ms;
        }),votes_.end());
        if(fresh) for(const auto &d:batch.items) {
            const bool signLeft=d.class_name=="left" || d.class_name=="no_right";
            const bool signRight=d.class_name=="right" || d.class_name=="no_left";
            double conf=0,area=0;
            if(d.class_name=="red") { conf=params.red_conf_min; area=params.red_box_min_area_frac; }
            else if(d.class_name=="green") { conf=params.green_conf_min; area=params.green_box_min_area_frac; }
            else if(d.class_name=="off") { conf=params.off_conf_min; area=params.off_box_min_area_frac; }
            else if(!(params.turn_enable && !turn_direction_ && !turn_fault_ && (signLeft || signRight))) continue;
            const auto &b=d.box;
            if(d.captured_ms!=batch.captured_ms || !std::isfinite(d.confidence) || d.confidence<conf || d.confidence>1 ||
               !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
               b.x<0 || b.y<0 || b.width<=0 || b.height<=0 || double(b.x)+b.width>1280 || double(b.y)+b.height>720 ||
               double(b.width)*b.height/(1280.0*720.0)<area) continue;
            if(d.class_name=="red") { red=true; red_x=std::max(red_x,(double(b.x)+b.width)/1280.0); }
            else if(d.class_name=="green") green=true;
            else if(d.class_name=="off") off=true;
            else { left=left || signLeft; right=right || signRight; }
        }
        if(!fresh || red || off || !green) green_run_=0;
        if(batch.frame_id>last_frame_) {
            last_frame_=batch.frame_id;
            if(fresh) {
                if(red) { red_ms_=batch.captured_ms; red_x_=red_x; }
                votes_.push_back({batch.captured_ms,red,green && !red && !off,left,right});
                if(!entered_wait && !input.stop && image_fresh && batch.captured_ms>wait_since_ms_ && green && !red && !off &&
                   (state_==PlannerState::CHO_DEN_XUAT_PHAT || state_==PlannerState::DUNG_DEN_DO)) ++green_run_;
            }
        }
        while(votes_.size()>size_t(params.vote_window)) votes_.pop_front();
        int reds=0,greens=0,lefts=0,rights=0;
        for(const auto &v:votes_) { reds+=v.red; greens+=v.green; lefts+=v.left; rights+=v.right; }
        if(params.turn_enable && !turn_direction_ && state_==PlannerState::BAM_LAN &&
           (lefts>=params.vote_min)!=(rights>=params.vote_min)) {
            turn_direction_=lefts>=params.vote_min ? 1 : -1; arm_odo_=input.odo_cm;
            edge_run_=0; edge_mask_ms_=-1;
            if(const char* error=sensorReason()) fault(error);
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
        if(released) reason_="DEN_XANH";
        else if(state_==PlannerState::BAM_LAN && red_fresh && reds>=params.vote_min) reason_="TIEP_CAN_DEN_DO";
        else if(state_==PlannerState::BAM_LAN) reason_="BAM_LAN";
    }
    bool returning=false;
    if(params.obstacle_action!=2 || state_!=PlannerState::BAM_LAN || entered_wait) {
        swerve_=Swerve::NONE; pass_cm_=0; sent_cms_=0; swerve_ms_=-1;
        if(params.obstacle_action==2) {
            obstacle_votes_.clear(); last_mask_ms_=-1; obstacle_clear_run_=0;
        }
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
        lane_.Reset(); returning=true;
    }
    if(params.turn_enable && turn_direction_ && state_!=PlannerState::DUNG && state_!=PlannerState::DUNG_DEN_DO &&
       state_!=PlannerState::DUNG_VAT_CAN) {
        if(!mask_fresh) {
            ClearTurn(); state_=PlannerState::BAM_LAN;
        } else if(turning_ && std::abs(input.yaw_deg-yaw0_)>=params.turn_yaw_deg) {
            ClearTurn(); state_=PlannerState::BAM_LAN; lane_.Reset();
        } else if(!turning_) {
            if(std::abs(input.odo_cm-arm_odo_)>params.turn_arm_cm) { ClearTurn(); cancelled=true; }
            else if(input.mask_ms!=edge_mask_ms_) {
                if(input.mask.dims!=2 || input.mask.type()!=CV_8UC1 || input.mask.cols<2)
                    throw std::runtime_error("Mặt nạ phải là CV_8UC1 H×W, W >= 2");
                int begin,end; bool surface;
                ScanLines(input.mask,std::min(input.mask.rows-1,int(params.scan_rows[0]*input.mask.rows)),begin,end,surface);
                edge_mask_ms_=input.mask_ms;
                edge_run_=surface && (turn_direction_==1 ? begin==0 : end==input.mask.cols) ? edge_run_+1 : 0;
                if(edge_run_>=2) {
                    turning_=true; yaw0_=input.yaw_deg; turn_odo_=progress_odo_=input.odo_cm;
                    turn_ms_=progress_ms_=now_ms;
                    state_=turn_direction_==1 ? PlannerState::RE_TRAI : PlannerState::RE_PHAI;
                }
            }
        }
    }
    plan.state=state_;
    if(params.turn_enable && turning_ && (state_==PlannerState::RE_TRAI || state_==PlannerState::RE_PHAI)) {
        const double step=params.steer_rate_dps*(command_ms_<0 ? 0 : std::max<int64_t>(0,now_ms-command_ms_)/1000.0);
        const double target=turn_direction_*params.turn_steer_deg;
        sent_angle_=std::max(-params.steer_max_deg,std::min(params.steer_max_deg,
            std::max(sent_angle_-step,std::min(sent_angle_+step,target))));
        command_ms_=now_ms;
        plan.cmd={params.min_cms,sent_angle_}; plan.reason=StateName(state_);
        plan.debug.state=plan.debug.reason=plan.reason;
        plan.debug.angle_deg=sent_angle_; plan.debug.speed_cms=plan.cmd.cms;
        return plan;
    }
    if(state_!=PlannerState::BAM_LAN) {
        sent_angle_=0; command_ms_=now_ms;
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
    if(params.obstacle_action==2) {
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
    if(cancelled) plan.reason=plan.debug.reason="HUY_RE";
    sent_angle_=plan.cmd.deg; command_ms_=now_ms;
    return plan;
}
}
