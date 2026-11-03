#include "app/Calibration.h"
#include "app/Safety.h"
#include "common/Smoke.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
namespace app {
bool CalibrationStep(const std::string &step) {
    return step=="servo" || step=="radius" || step=="motor" || step=="position" || step=="imu" ||
           step=="tof" || step=="camera" || step=="latency" || step=="timing";
}
Calibration::Calibration(const common::Config &c,std::string step,std::string dir,std::string config):c_(c),step_(std::move(step)),directory_(std::move(dir)),
        config_(std::move(config)) {
    config_text_=c_.loaded_text.empty() ? common::ConfigText(config_) : c_.loaded_text;
    speed_=std::min(c_.calib_speed_cms,c_.max_speed_cms);
    std::cout<<"Hiệu chuẩn "<<step_<<": q thoát; x/Space dừng; nhập phím rồi Enter ở terminal.\n"
        <<"ToF: che từng cảm biến, đối chiếu slot và tên; servo: bánh kê, xác nhận chiều bằng mắt.\n"
        <<"a/d đổi góc theo calib_servo_step_deg; +/- đổi tốc độ theo keyboard_speed_step_cms; "
          "y bắt đầu lượt có giới hạn; c chụp ảnh; m đánh dấu độ trễ.\n"<<std::flush;
}
void Calibration::BeginMotion(const common::Observation &o) {
    active_=true; start_cm_=o.odom_cm; start_yaw_=o.yaw_rel_deg;
    motion_limits_.enabled=true;
    motion_limits_.deadline=now_+std::chrono::duration_cast<common::Clock::duration>(
        std::chrono::duration<double>(c_.calib_max_duration_s));
    motion_limits_.start_odom_cm=start_cm_;
    motion_limits_.max_distance_cm=step_=="position" ? c_.calib_position_cm : c_.calib_distance_cm;
    motion_limits_.limit_yaw=step_=="radius";
    motion_limits_.start_yaw_deg=start_yaw_;
}
common::ControlCommand Calibration::Tick(const common::Snapshot &s,int key,common::Clock::time_point now) {
    if(done_) return {};
    now_=now; const auto &o=s.observation;
    const double elapsed=std::chrono::duration<double>(now-started_).count();
    const bool fresh_odom=o.odom_valid && std::isfinite(o.odom_cm) && o.odom_at!=common::Clock::time_point{} &&
        o.odom_at<=now && now-o.odom_at<std::chrono::milliseconds(c_.tof_timeout_ms);
    if(key=='q' || key=='Q' || key==27) { done_=true; return {}; }
    if(key=='x' || key=='X' || key==' ') {
        active_=false; angle_=0;
        return {};
    }
    if(now-printed_>=std::chrono::seconds(1)) {
        printed_=now;
        std::ostringstream line; // một lần ghi, không chen vào dòng AN_TOAN_CHOT của luồng chấp hành
        line<<"t="<<elapsed<<" encoder="<<o.odom_cm<<" cm; v="<<o.meas_speed
            <<" yaw_rel="<<o.yaw_rel_deg<<" pitch="<<o.pitch_deg<<" Euler="<<o.imu.eulerDeg.x<<'/'
            <<o.imu.eulerDeg.y<<'/'<<o.imu.eulerDeg.z<<" FPS="<<s.fps<<" Actuator_ms="<<s.actuator_ms
            <<" IMU calib="<<int(o.imu.calibration.system)<<'/'<<int(o.imu.calibration.gyroscope)<<'/'
            <<int(o.imu.calibration.accelerometer)<<'/'<<int(o.imu.calibration.magnetometer)<<'\n';
        for(int i=0;i<4;++i) line<<"slot "<<i<<' '<<c_.tof_names[i]<<'='<<o.tof[i].distanceMm
            <<" valid="<<o.tof[i].valid<<" tuổi_ms="<<std::chrono::duration<double,std::milli>(now-o.tof[i].updatedAt).count()<<"; ";
        line<<'\n'; std::cout<<line.str()<<std::flush;
    }
    if(key=='a') angle_-=c_.calib_servo_step_deg;
    if(key=='d') angle_+=c_.calib_servo_step_deg;
    angle_=std::clamp(angle_,-std::min(c_.calib_servo_limit_deg,c_.max_steering_deg),std::min(c_.calib_servo_limit_deg,c_.max_steering_deg));
    if(key=='+') speed_+=c_.keyboard_speed_step_cms;
    if(key=='-') speed_-=c_.keyboard_speed_step_cms;
    speed_=std::clamp(speed_,0.0f,c_.max_speed_cms);
    if(key=='c' && !o.frame.empty()) {
        if(!cv::imwrite(directory_+"/calib_"+std::to_string(o.frame_id)+".jpg",o.frame)) throw std::runtime_error("Không lưu được ảnh hiệu chuẩn");
    }
    if(key=='m' && step_=="latency") {
        ++marker_;
        angle_=angle_>0 ? -c_.calib_servo_step_deg : c_.calib_servo_step_deg;
        std::ofstream mark(directory_+"/markers.csv",std::ios::app);
        mark<<elapsed<<','<<o.frame_id<<','<<angle_<<','<<marker_<<'\n'; mark.close();
        if(!mark) throw std::runtime_error("Không ghi được mốc độ trễ");
        state_="DANH_DAU_"+std::to_string(marker_);
    }
    const bool motion_step=step_=="motor" || step_=="radius" || step_=="position" || step_=="servo";
    const bool fresh_image=FreshImage(c_,o,now);
    if(!fresh_image || s.state=="MAT_ANH") CancelMotion();
    if(key=='y' && motion_step && !active_) {
        if(!c_.tof_mapping_confirmed || !fresh_odom || !fresh_image) std::cerr<<"Chưa xác nhận ToF hoặc encoder/ảnh không mới: giữ dừng.\n";
        else {
            BeginMotion(o);
        }
    }
    const double distance=std::abs(o.odom_cm-start_cm_),yaw=std::abs(o.yaw_rel_deg-start_yaw_);
    if(active_ && (CalibrationLimitReached(c_,o,motion_limits_,now) ||
        s.state=="PHANH_TOF" || s.state=="PHANH_TOF_SAU")) {
        CancelMotion();
        std::ostringstream stop_line;
        stop_line<<"Dừng lượt: quãng đường="<<distance<<" yaw="<<yaw<<" bán kính cm="<<(yaw>0 ? distance/(yaw*3.141592653589793/180) : 0)<<'\n';
        std::cout<<stop_line.str();
    }
    if(state_.empty()) state_="HIEU_CHUAN_"+step_;
    common::ControlCommand cmd{active_ ? speed_ : 0,angle_};
    cmd.position_control=active_ && step_=="position"; cmd.position_cm=c_.calib_position_cm;
    cmd.marker=marker_;
    if(active_) cmd.calib_limits=motion_limits_;
    return cmd;
}
}
