#include "control/Planner.h"

namespace control {
const char *StateName(PlannerState state) {
    switch(state) {
        case PlannerState::CHO_SAN_SANG: return "CHO_SAN_SANG";
        case PlannerState::BAM_LAN: return "BAM_LAN";
        case PlannerState::DUNG: return "DUNG";
    }
    return "DUNG";
}
void Planner::Reset() { lane_.Reset(); state_=PlannerState::CHO_SAN_SANG; reason_="CHO_G"; }
Plan Planner::Step(const PlannerInput &input, int64_t now_ms, const ControlParams &params) {
    params.Validate();
    Plan plan;
    plan.reference=input.reference;
    const bool image_fresh=input.image_ms>=0 && input.image_ms<=now_ms && now_ms-input.image_ms<=kStaleImageMs;
    if(input.stop) { state_=PlannerState::DUNG; reason_="SPACE"; lane_.Reset(); }
    else if(state_==PlannerState::BAM_LAN && !image_fresh) {
        state_=PlannerState::DUNG; reason_=kMissingImageReason; lane_.Reset();
    }
    else if(input.go && state_!=PlannerState::BAM_LAN) {
        const bool mask_fresh=!input.mask.empty() && input.mask_ms>=0 && input.mask_ms<=now_ms &&
                              now_ms-input.mask_ms<=params.mask_max_age_ms;
        if(!image_fresh || !mask_fresh) {
            plan.state=state_;
            plan.reason=image_fresh ? "go ignored MAT_NAN_CU" : "go ignored MAT_ANH";
            plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
            return plan;
        }
        state_=PlannerState::BAM_LAN; reason_="BAM_LAN"; lane_.Reset();
    }
    plan.state=state_;
    if(state_!=PlannerState::BAM_LAN) {
        plan.reason=reason_;
        plan.debug.state=StateName(state_); plan.debug.reason=plan.reason;
        return plan;
    }
    const auto result=lane_.Step(input.mask,input.mask_ms,now_ms,params,input.reference);
    plan.cmd=result.cmd; plan.debug=result.debug; plan.reason=result.debug.reason;
    return plan;
}
}
