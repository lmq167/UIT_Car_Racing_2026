#pragma once
#include "common/Config.h"
#include "common/Types.h"
#include <algorithm>
#include <cmath>
namespace app {
inline bool FreshImage(const common::Config &c,const common::Observation &o,common::Clock::time_point now) {
    return o.frame_id && now-o.captured_at<std::chrono::milliseconds(c.image_timeout_ms);
}
inline bool CalibrationLimitReached(const common::Config &c,const common::Observation &o,
        const common::CalibrationLimits &limits,common::Clock::time_point now) {
    if(!limits.enabled) return false;
    return now>=limits.deadline || !o.odom_valid || !std::isfinite(o.odom_cm) ||
        o.odom_at==common::Clock::time_point{} || o.odom_at>now ||
        now-o.odom_at>=std::chrono::milliseconds(c.tof_timeout_ms) ||
        std::abs(o.odom_cm-limits.start_odom_cm)>=limits.max_distance_cm ||
        (limits.limit_yaw && (!std::isfinite(o.yaw_rel_deg) ||
            std::abs(o.yaw_rel_deg-limits.start_yaw_deg)>=limits.max_yaw_deg));
}
inline common::ControlCommand SafeCommand(const common::Config &c, const common::Observation &o,
        common::ControlCommand cmd, common::Clock::time_point now, std::string &state) {
    if(!std::isfinite(cmd.speedCmPerSec) || (cmd.position_control && !std::isfinite(cmd.position_cm))) cmd={};
    cmd.speedCmPerSec=std::clamp(cmd.speedCmPerSec,-c.max_speed_cms,c.max_speed_cms);
    cmd.steeringDeg=std::clamp(cmd.steeringDeg,-c.max_steering_deg,c.max_steering_deg);
    if(!FreshImage(c,o,now)) {
        cmd={}; state="MAT_ANH";
    }
    const bool reverse=cmd.speedCmPerSec<0;
    const auto &front=o.tof[reverse ? c.RearSlot() : c.FrontSlot()];
    if(c.tof_safety_enabled && cmd.speedCmPerSec!=0 &&
       (!front.valid || front.timedOut || front.distanceMm<0 ||
        now-front.updatedAt>std::chrono::milliseconds(c.tof_timeout_ms) || front.distanceMm<c.tof_brake_mm)) {
        cmd.speedCmPerSec=0; state=reverse ? "PHANH_TOF_SAU" : "PHANH_TOF";
    }
    return cmd;
}
// Trạng thái chỉ thuộc luồng Actuator; đo khoảng mất ảnh từ khung cuối cùng.
class CameraRecovery {
    common::Clock::time_point last_{}, last_frame_{}, stable_{}, ramp_{};
    bool gap_=false, waiting_stable_=false, ramping_=false;
public:
    common::ControlCommand Filter(const common::Config &c,const common::Observation &o,
            common::ControlCommand cmd,common::Clock::time_point now,std::string &state) {
        const auto timeout=std::chrono::milliseconds(c.image_timeout_ms);
        if(!FreshImage(c,o,now)) {
            if(o.frame_id) { if(!gap_) last_=o.captured_at; gap_=true; }
            if(gap_ && std::chrono::duration<double>(now-last_).count()>c.camera_resume_max_s) waiting_stable_=true;
            stable_={};
            ramping_=false;
            state="MAT_ANH";
            return {};
        }
        // Vẫn nhận ra khoảng ngắt nếu vòng chấp hành chưa kịp quan sát ảnh cũ.
        if(!gap_ && last_frame_!=common::Clock::time_point{} && o.captured_at-last_frame_>=timeout) {
            last_=last_frame_; gap_=true; stable_={}; ramping_=false;
        }
        last_frame_=o.captured_at;
        if(gap_) {
            if(std::chrono::duration<double>(o.captured_at-last_).count()>c.camera_resume_max_s) waiting_stable_=true;
            gap_=false; ramp_=o.captured_at; ramping_=true;
        }
        if(waiting_stable_) {
            if(stable_==common::Clock::time_point{}) stable_=o.captured_at;
            // Đo tới khung mới nhất: giữ nguyên một khung không được tính là ảnh liên tục.
            if(std::chrono::duration<double>(o.captured_at-stable_).count()<c.resume_stable_s) {
                state="DUNG_CHO_ANH_ON_DINH"; return {};
            }
            waiting_stable_=false; stable_={}; ramp_=now; ramping_=true;
        }
        if(ramping_) {
            const double ratio=std::clamp(std::chrono::duration<double>(now-ramp_).count()/c.soft_start_s,0.0,1.0);
            cmd.speedCmPerSec*=static_cast<float>(ratio);
            if(ratio<1) state="TANG_TOC_MEM"; else ramping_=false;
        }
        return cmd;
    }
};
}
