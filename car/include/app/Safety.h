#pragma once
#include "common/Config.h"
#include "common/Types.h"
#include <algorithm>
#include <cmath>
namespace app {
inline common::ControlCommand SafeCommand(const common::Config &c, const common::Observation &o,
        common::ControlCommand cmd, common::Clock::time_point now, std::string &state) {
    if(!std::isfinite(cmd.speedCmPerSec)) cmd={};
    cmd.speedCmPerSec=std::clamp(cmd.speedCmPerSec,-c.max_speed_cms,c.max_speed_cms);
    cmd.steeringDeg=std::clamp(cmd.steeringDeg,-c.max_steering_deg,c.max_steering_deg);
    if(!o.frame_id || now-o.captured_at>std::chrono::milliseconds(c.image_timeout_ms)) {
        cmd={}; state="MAT_ANH";
    }
    const auto &front=o.tof[c.FrontSlot()];
    if(c.tof_safety_enabled && cmd.speedCmPerSec>0 &&
       (!front.valid || front.timedOut || front.distanceMm<0 ||
        now-front.updatedAt>std::chrono::milliseconds(c.tof_timeout_ms) || front.distanceMm<c.tof_brake_mm)) {
        cmd.speedCmPerSec=0; state="PHANH_TOF";
    }
    return cmd;
}
}
