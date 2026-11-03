#pragma once
#include "common/Types.h"
#include "common/Config.h"
#include <string>
namespace app {
class Calibration {
public:
    Calibration(const common::Config &c,std::string step,std::string directory,std::string config);
    common::ControlCommand Tick(const common::Snapshot &s,int key,common::Clock::time_point now=common::Clock::now());
    void CancelMotion() {
        active_=false; angle_=0;
    }
    bool Done() const { return done_; }
    bool Failed() const { return failed_; }
    std::string State() const { return state_; }
private:
    void BeginMotion(const common::Observation &o);
    common::Config c_;
    std::string step_,directory_,config_,state_,config_text_;
    common::Clock::time_point started_=common::Clock::now(), printed_{};
    common::Clock::time_point now_=started_;
    int angle_=0;
    std::uint64_t marker_=0;
    float speed_=0;
    double start_cm_=0,start_yaw_=0;
    common::CalibrationLimits motion_limits_{};
    bool active_=false,done_=false,failed_=false;
};
bool CalibrationStep(const std::string &step);
}
