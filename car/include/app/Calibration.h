#pragma once
#include "common/Types.h"
#include "common/Config.h"
#include <string>
namespace app {
class Calibration {
public:
    Calibration(const common::Config &c,std::string step,std::string directory,std::string config,
                std::string kill_file,bool drop_camera);
    common::ControlCommand Tick(const common::Snapshot &s,int key,common::Clock::time_point now=common::Clock::now());
    void CancelMotion() {
        active_=false; angle_=0;
        if(step_=="smoke") { failed_=done_=true; dropping_=false; }
    }
    bool Done() const { return done_; }
    bool Failed() const { return failed_; }
    bool DropCamera() const { return dropping_; }
    std::string State() const { return state_; }
private:
    void Result(bool killed=false,bool stopped=false);
    void Next(int stage,const char *prompt);
    void BeginMotion(const common::Observation &o);
    common::Config c_;
    std::string step_,directory_,config_,kill_file_,state_,config_text_;
    common::Clock::time_point started_=common::Clock::now(), stage_at_=started_, printed_{};
    common::Clock::time_point now_=started_;
    int stage_=0, angle_=0;
    std::uint64_t marker_=0;
    float speed_=0;
    double start_cm_=0,start_yaw_=0;
    common::CalibrationLimits motion_limits_{};
    bool active_=false,done_=false,failed_=false,dropping_=false,drop_enabled_=false;
    bool camera_=false,sensors_=false,servo_=false,tof_=false,drop_=false,moving_=false;
};
bool CalibrationStep(const std::string &step);
}
