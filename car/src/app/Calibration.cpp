#include "app/Calibration.h"
#include "app/Safety.h"
#include "common/Smoke.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>
namespace app {
bool CalibrationStep(const std::string &step) {
    return step=="servo" || step=="radius" || step=="motor" || step=="position" || step=="imu" ||
           step=="tof" || step=="camera" || step=="latency" || step=="timing";
}
Calibration::Calibration(const common::Config &c,std::string step,std::string dir,std::string config,
        std::string kill_file,bool drop_camera):c_(c),step_(std::move(step)),directory_(std::move(dir)),
        config_(std::move(config)),kill_file_(std::move(kill_file)),drop_enabled_(drop_camera) {
    config_text_=c_.loaded_text.empty() ? common::ConfigText(config_) : c_.loaded_text;
    speed_=std::min(c_.calib_speed_cms,c_.max_speed_cms);
    std::cout<<"Hiệu chuẩn "<<step_<<": q thoát; x/Space dừng; nhập phím rồi Enter ở terminal.\n"
        <<"ToF: che từng cảm biến, đối chiếu slot và tên; servo: bánh kê, xác nhận chiều bằng mắt.\n"
        <<"a/d đổi góc theo calib_servo_step_deg; +/- đổi tốc độ theo keyboard_speed_step_cms; "
          "y bắt đầu lượt có giới hạn; c chụp ảnh; m đánh dấu độ trễ.\n"<<std::flush;
    if(step_=="smoke") {
        Result();
        common::SmokeResult(c_.log_dir+"/smoke_gate.json",config_text_,false,false,false,false,false,false,false,false);
        std::cout<<"Smoke: kiểm camera và cảm biến trước; mọi xác nhận dùng phím y.\n"<<std::flush;
        if(!c_.tof_mapping_confirmed) {
            std::cerr<<"Ánh xạ chưa xác nhận: dùng --mode calib tof, sửa tof_names trong config.yaml, "
                "xác nhận chiều servo rồi đặt tof_mapping_confirmed: 1; dừng ở bước 3, không chạy motor.\n";
        }
    }
}
void Calibration::Result(bool killed,bool stopped) {
    common::SmokeResult(directory_+"/smoke.json",config_text_,camera_,sensors_,servo_,tof_,drop_,moving_,killed,stopped);
    if(done_) {
        std::cout<<"Mục | Kết quả\n";
        for(const auto &item:{std::make_pair("Camera",camera_),{"IMU/ToF",sensors_},{"Servo",servo_},
                {"Phanh ToF",tof_},{"Phanh mất ảnh",drop_},{"Encoder đang chạy",moving_},{"Kill -9 và encoder dừng",killed&&stopped}})
            std::cout<<item.first<<" | "<<(item.second ? "ĐẠT" : "KHÔNG ĐẠT")<<'\n';
    }
}
void Calibration::Next(int stage,const char *prompt) {
    stage_=stage; stage_at_=now_; std::cout<<prompt<<std::endl;
}
common::ControlCommand Calibration::Tick(const common::Snapshot &s,int key,common::Clock::time_point now) {
    if(done_) return {};
    now_=now; const auto &o=s.observation;
    const double elapsed=std::chrono::duration<double>(now-started_).count();
    const auto phase=[&]{ return std::chrono::duration<double>(now-stage_at_).count(); };
    const bool fresh_odom=o.odom_valid && now-o.odom_at<std::chrono::milliseconds(c_.tof_timeout_ms);
    if(key=='q' || key=='Q' || key==27) { done_=true; if(step_=="smoke") { failed_=true; Result(); } return {}; }
    if(key=='x' || key=='X' || key==' ') {
        active_=false; angle_=0;
        if(step_=="smoke") { failed_=done_=true; Result(); }
        return {};
    }
    if(now-printed_>=std::chrono::seconds(1)) {
        printed_=now;
        std::cout<<"t="<<elapsed<<" encoder="<<o.odom_cm<<" cm; v="<<o.meas_speed
            <<" yaw_rel="<<o.yaw_rel_deg<<" pitch="<<o.pitch_deg<<" Euler="<<o.imu.eulerDeg.x<<'/'
            <<o.imu.eulerDeg.y<<'/'<<o.imu.eulerDeg.z<<" FPS="<<s.fps<<" Actuator_ms="<<s.actuator_ms
            <<" IMU calib="<<int(o.imu.calibration.system)<<'/'<<int(o.imu.calibration.gyroscope)<<'/'
            <<int(o.imu.calibration.accelerometer)<<'/'<<int(o.imu.calibration.magnetometer)<<'\n';
        for(int i=0;i<4;++i) std::cout<<"slot "<<i<<' '<<c_.tof_names[i]<<'='<<o.tof[i].distanceMm
            <<" valid="<<o.tof[i].valid<<" tuổi_ms="<<std::chrono::duration<double,std::milli>(now-o.tof[i].updatedAt).count()<<"; ";
        std::cout<<std::endl;
    }
    if(step_=="smoke") {
        if(elapsed>120) { failed_=done_=true; Result(); std::cerr<<"KHÔNG ĐẠT: hết 120 s\n"; return {}; }
        state_="SMOKE_"+std::to_string(stage_);
        if(stage_==0 && elapsed>=3) {
            camera_=o.frame_id && now-o.captured_at<std::chrono::milliseconds(c_.image_timeout_ms) &&
                o.frame.cols==c_.camera_width && o.frame.rows==c_.camera_height && s.fps>=c_.smoke_min_fps;
            sensors_=o.imu_valid && now-o.sensor_at<std::chrono::milliseconds(c_.tof_timeout_ms);
            for(const auto &t:o.tof) sensors_=sensors_&&t.valid&&!t.timedOut&&t.distanceMm>=0&&now-t.updatedAt<std::chrono::milliseconds(c_.tof_timeout_ms);
            Result();
            if(!camera_ || !sensors_ || !c_.tof_mapping_confirmed) {
                std::cerr<<"KHÔNG ĐẠT: camera/cảm biến/ánh xạ. Che từng ToF bằng --mode calib tof; "
                    "sửa tof_names trong config.yaml, xác nhận chiều servo khi bánh kê; đặt tof_mapping_confirmed: 1. Dừng ở bước 3, không chạy motor.\n";
                failed_=done_=true; return {};
            }
            Next(1,"Servo ở trim: quan sát, y xác nhận (bánh đã kê).");
        }
        if(key=='y' && stage_>=1 && stage_<=4) {
            if(stage_==1) Next(2,"Servo lệch âm nhẹ: y xác nhận chiều theo cấu hình.");
            else if(stage_==2) Next(3,"Servo lệch dương nhẹ: y xác nhận chiều theo cấu hình.");
            else if(stage_==3) Next(4,"Servo về trim: y xác nhận.");
            else { servo_=true; Result(); Next(5,"Bánh đã kê lên khỏi mặt đất? y xác nhận; motor chạy 2 s ở tối đa 10 cm/s."); }
        } else if(key=='y' && stage_==5) {
            if(kill_file_.empty()) {
                failed_=done_=true; Result();
                std::cerr<<"Chạy scripts/smoke_test.sh --smoke-drop-camera để có phép kill -9; chưa chạy motor.\n";
                return {};
            }
            Next(6,"Đang kiểm encoder > 0; không chạm bánh.");
        }
        const float test_speed=std::min(10.0f,c_.max_speed_cms);
        const auto &front=o.tof[c_.FrontSlot()];
        if(stage_==6 && phase()>=2) {
            if(!fresh_odom || o.meas_speed<=c_.smoke_stop_speed_cms) { failed_=done_=true; Result(); return {}; }
            Next(7,"Đưa tay trước ToF front < tof_brake_mm (không chạm bánh), giữ tới khi encoder dừng.");
        }
        if(stage_==7 && front.valid && front.distanceMm>=0 && front.distanceMm<c_.tof_brake_mm)
            Next(8,"Đang đo thời gian phanh ToF bằng encoder.");
        if(stage_==8) {
            if(fresh_odom && std::abs(o.meas_speed)<=c_.smoke_stop_speed_cms && phase()<=1) {
                tof_=true; Result(); Next(9,"Rút tay khỏi front rồi y: kiểm mất ảnh nhân tạo bằng --smoke-drop-camera.");
            } else if(phase()>1) { failed_=done_=true; Result(); return {}; }
        }
        if(stage_==9 && key=='y') {
            if(!drop_enabled_) { failed_=done_=true; Result(); return {}; }
            Next(10,"Chạy lại trước khi ngắt ảnh nhân tạo.");
        }
        if(stage_==10 && phase()>=1 && fresh_odom && o.meas_speed>c_.smoke_stop_speed_cms) {
            dropping_=true; Next(11,"Đã ngắt khung ảnh; đo encoder dừng trong <= 1 s.");
        }
        if(stage_==11) {
            if(phase()>c_.image_timeout_ms/1000.0 && fresh_odom && std::abs(o.meas_speed)<=c_.smoke_stop_speed_cms && phase()<=1) {
                dropping_=false; drop_=true; Result(); Next(12,"Chạy lại trước phép kill -9 độc lập.");
            } else if(phase()>1) { failed_=done_=true; Result(); return {}; }
        }
        if(stage_==12 && phase()>=1 && fresh_odom && o.meas_speed>c_.smoke_stop_speed_cms) {
            moving_=true; Result();
            const auto temporary=kill_file_+".tmp";
            std::ofstream out(temporary); out<<directory_<<'\n'; out.close();
            if(!out || std::rename(temporary.c_str(),kill_file_.c_str())!=0)
                throw std::runtime_error("Không báo được yêu cầu kill -9 cho run.sh");
            Next(13,"run.sh sẽ kill -9 rồi ucr_stop đo encoder; chưa kết luận ĐẠT.");
        }
        if(stage_==2) return {0,-std::min(c_.smoke_servo_deg,c_.max_steering_deg)};
        if(stage_==3) return {0,std::min(c_.smoke_servo_deg,c_.max_steering_deg)};
        if(stage_==6 || stage_==7 || stage_==8 || stage_==10 || stage_==11 || stage_==12 || stage_==13) return {test_speed,0};
        return {};
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
    if(key=='y' && motion_step) {
        if(!c_.tof_mapping_confirmed || !fresh_odom || !fresh_image) std::cerr<<"Chưa xác nhận ToF hoặc encoder/ảnh không mới: giữ dừng.\n";
        else { active_=true; start_cm_=o.odom_cm; start_yaw_=o.yaw_rel_deg; motion_at_=now; }
    }
    const double distance=std::abs(o.odom_cm-start_cm_),yaw=std::abs(o.yaw_rel_deg-start_yaw_);
    if(active_ && (!fresh_odom || now-motion_at_>=std::chrono::duration<double>(c_.calib_max_duration_s) ||
        (step_=="radius" ? yaw>=360 : distance>=(step_=="position" ? c_.calib_position_cm : c_.calib_distance_cm)) ||
        s.state=="PHANH_TOF" || s.state=="PHANH_TOF_SAU")) {
        active_=false;
        std::cout<<"Dừng lượt: quãng đường="<<distance<<" yaw="<<yaw<<" bán kính cm="<<(yaw>0 ? distance/(yaw*3.141592653589793/180) : 0)<<'\n';
    }
    if(state_.empty()) state_="HIEU_CHUAN_"+step_;
    common::ControlCommand cmd{active_ ? speed_ : 0,angle_};
    cmd.position_control=active_ && step_=="position"; cmd.position_cm=c_.calib_position_cm;
    cmd.marker=marker_;
    return cmd;
}
}
