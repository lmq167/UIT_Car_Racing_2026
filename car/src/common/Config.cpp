#include "common/Config.h"
#include "common/ConfigValue.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <set>
namespace common {
void Config::Load(const std::string &path) {
    std::ifstream input(path,std::ios::binary);
    if(!input) throw std::runtime_error("không đọc được: " + path);
    std::ostringstream text; text<<input.rdbuf(); loaded_text=text.str();
    // Cấu hình đã chạy và bằng chứng smoke phải là cùng một bản chụp nội dung.
    cv::FileStorage fs;
    try { fs.open(loaded_text, cv::FileStorage::READ|cv::FileStorage::MEMORY); }
    catch(const cv::Exception &) { throw std::runtime_error("không đọc được: " + path); }
    if (!fs.isOpened()) throw std::runtime_error("không đọc được: " + path);
    std::set<std::string> known_keys;
    auto read = [&](const char *name, auto &value, const std::string &range) {
        known_keys.insert(name);
        if (!HasConfigKey(fs.root(),name)) std::cerr << "Thieu khoa, dung mac dinh: " << name << '\n';
        else {
            try { ReadConfigValue(name,fs[name],value); }
            catch(const std::runtime_error &) {
                throw std::runtime_error(std::string(name)+" = "+ConfigValueText(fs[name])+" ngoài khoảng hợp lệ "+range);
            }
        }
    };
    auto flag = [&](const char *name, int &value) {
        known_keys.insert(name);
        const auto node=fs[name];
        if(!HasConfigKey(fs.root(),name)) { std::cerr << "Thieu khoa, dung mac dinh: " << name << '\n'; return; }
        try { ReadConfigFlag(name,node,value); }
        catch(const std::runtime_error &) {
            throw std::runtime_error(std::string(name)+" = "+ConfigValueText(node)+" ngoài khoảng hợp lệ {0, 1} (số nguyên)");
        }
    };
    auto require = [&](const char *name,bool valid,const std::string &range) {
        if(!valid) throw std::runtime_error(std::string(name)+" = "+ConfigValueText(fs[name])+" ngoài khoảng hợp lệ "+range);
    };
    read("camera_pipeline", camera_pipeline, "chuỗi GStreamer");
    read("fake_source", fake_source, "chuỗi (cho phép rỗng)");
    read("camera_hz", camera_hz, "[1, +inf) Hz");
    read("sensor_hz", sensor_hz, "[10, +inf) Hz");
    read("control_hz", control_hz, "[10, +inf) Hz");
    read("odometry_hz", odometry_hz, "[1, +inf) Hz");
    read("ui_hz", ui_hz, "[1, +inf) Hz");
    double speed_limit=max_speed_cms;
    read("max_speed_cms", speed_limit, "(0, 100] cm/s");
    // Kiểm trước khi ép float để số vừa lớn hơn 100 không bị làm tròn xuống 100.
    if(!std::isfinite(speed_limit) || speed_limit<=0 || speed_limit>100 || static_cast<float>(speed_limit)==0)
        throw std::runtime_error("max_speed_cms = "+ConfigValueText(fs["max_speed_cms"])+" ngoài khoảng hợp lệ (0, 100] cm/s");
    max_speed_cms=static_cast<float>(speed_limit);
    read("max_steering_deg", max_steering_deg, "[0, 60 - |servo_trim_deg|] độ");
    read("servo_trim_deg", servo_trim_deg, "[-60, 60] độ");
    flag("servo_invert", servo_invert);
    flag("tof_safety_enabled", tof_safety_enabled);
    read("tof_brake_mm", tof_brake_mm, "[1, +inf) mm");
    read("tof_timeout_ms", tof_timeout_ms, "[1, 500] ms");
    read("image_timeout_ms", image_timeout_ms, "[1, 500] ms");
    read("camera_resume_max_s", camera_resume_max_s, "> image_timeout_ms/1000 s");
    read("soft_start_s", soft_start_s, "(0, +inf) s");
    double stable_seconds=resume_stable_s;
    read("resume_stable_s", stable_seconds, "[0.5, 30] s");
    require("resume_stable_s",stable_seconds>=0.5 && stable_seconds<=30,"[0.5, 30] s");
    resume_stable_s=static_cast<float>(stable_seconds);
    flag("tof_mapping_confirmed",tof_mapping_confirmed);
    read("camera_width", camera_width, "[1, +inf)"); read("camera_height", camera_height, "[1, +inf)");
    read("smoke_min_fps", smoke_min_fps, "(0, +inf) FPS"); read("smoke_stop_speed_cms", smoke_stop_speed_cms, "(0, +inf) cm/s");
    read("smoke_servo_deg", smoke_servo_deg, "[1, 60] độ");
    read("calib_speed_cms", calib_speed_cms, "(0, +inf) cm/s");
    read("calib_servo_step_deg", calib_servo_step_deg, "[1, +inf) độ"); read("calib_servo_limit_deg", calib_servo_limit_deg, "[1, 60] độ");
    auto calib_limit = [&](const char *name,float &value,double maximum,const char *range) {
        double number=value;
        read(name,number,range);
        // Kiểm trước khi ép float để không làm tròn số vượt trần thành hợp lệ.
        require(name,number>0 && number<=maximum && static_cast<float>(number)>0,range);
        value=static_cast<float>(number);
    };
    calib_limit("calib_distance_cm",calib_distance_cm,300,"(0, 300] cm");
    calib_limit("calib_position_cm",calib_position_cm,100,"(0, 100] cm");
    calib_limit("calib_max_duration_s",calib_max_duration_s,120,"(0, 120] s");
    require("image_timeout_ms",image_timeout_ms>=1 && image_timeout_ms<=500,"[1, 500] ms");
    require("camera_resume_max_s",camera_resume_max_s>image_timeout_ms/1000.0f,"> image_timeout_ms/1000 s");
    require("soft_start_s",soft_start_s>0,"(0, +inf) s");
    require("smoke_servo_deg",smoke_servo_deg>=1 && smoke_servo_deg<=60,"[1, 60] độ");
    require("camera_width",camera_width>=1,"[1, +inf)");
    require("camera_height",camera_height>=1,"[1, +inf)");
    require("smoke_min_fps",smoke_min_fps>0,"(0, +inf)");
    require("smoke_stop_speed_cms",smoke_stop_speed_cms>0,"(0, +inf) cm/s");
    require("calib_speed_cms",calib_speed_cms>0,"(0, +inf) cm/s");
    require("calib_servo_step_deg",calib_servo_step_deg>=1,"[1, +inf) độ");
    require("calib_servo_limit_deg",calib_servo_limit_deg>=1 && calib_servo_limit_deg<=60,"[1, 60] độ");
    read("imu_calibration_file", imu_calibration_file, "chuỗi đường dẫn");
    read("imu_drift_ms", imu_drift_ms, "[1, +inf) ms");
    read("yaw_axis", yaw_axis, "[0, 2]");
    read("pitch_axis", pitch_axis, "[0, 2]");
    read("roll_axis", roll_axis, "[0, 2]");
    read("yaw_sign", yaw_sign, "{-1, 1}");
    read("pitch_sign", pitch_sign, "{-1, 1}");
    read("roll_sign", roll_sign, "{-1, 1}");
    read("log_dir", log_dir, "chuỗi đường dẫn");
    read("image_every_n", image_every_n, "[1, +inf)");
    read("jpeg_quality", jpeg_quality, "[1, 100]");
    read("image_queue_size", image_queue_size, "[1, +inf)");
    read("speed_filter_size", speed_filter_size, "[1, +inf)");
    flag("enable_oled", enable_oled);
    flag("enable_display", enable_display);
    read("keyboard_speed_step_cms", keyboard_speed_step_cms, "(0, +inf) cm/s");
    read("keyboard_steering_step_deg", keyboard_steering_step_deg, "[1, +inf) độ");
    read("motor_keepalive_ms", motor_keepalive_ms, "[1, 200] ms");
    read("fps_report_ms", fps_report_ms, "[1, +inf) ms");
    read("collect_fps", collect_fps, "[1, 10]");
    read("collect_min_speed_cms", collect_min_speed_cms, "(0, +inf) cm/s");
    read("collect_duplicate_threshold", collect_duplicate_threshold, "[0, 255]");
    read("collect_confirm_ms", collect_confirm_ms, "[1, +inf) ms");
    read("collect_keys", collect_keys, "đúng 12 phím ASCII thường hoặc esc, không trùng nhau");
    read("collect_luot_keys", collect_luot_keys, "đúng 9 phím ASCII thường, không trùng nhau hoặc collect_keys");
    require("collect_fps",collect_fps>=1 && collect_fps<=10,"[1, 10]");
    require("collect_min_speed_cms",std::isfinite(collect_min_speed_cms) && collect_min_speed_cms>0,"(0, +inf) cm/s");
    require("collect_duplicate_threshold",std::isfinite(collect_duplicate_threshold) &&
        collect_duplicate_threshold>=0 && collect_duplicate_threshold<=255,"[0, 255]");
    require("collect_confirm_ms",collect_confirm_ms>=1,"[1, +inf) ms");
    require("collect_keys",collect_keys.size()==12,"đúng 12 phím");
    require("collect_luot_keys",collect_luot_keys.size()==9,"đúng 9 phím");
    std::string used;
    auto add_key=[&](const std::string &key,const char *name) {
        const char ch=key=="esc" ? 27 : (key.size()==1 ? key[0] : 0);
        require(name,ch && (ch==27 || (ch>=32 && ch<=126 && !(ch>='A'&&ch<='Z'))) &&
            used.find(ch)==std::string::npos,"ký tự ASCII thường hoặc esc; không trùng nhau giữa các bảng phím");
        used+=ch;
    };
    for(const auto &key:collect_keys) add_key(key,"collect_keys");
    for(char ch:collect_luot_keys) add_key(std::string(1,ch),"collect_luot_keys");
    read("tof_names", tof_names, "front, rear, left, right, mỗi tên một lần");
    require("tof_names",tof_names.size()==4,"đúng 4 slot: front, rear, left, right, mỗi tên một lần");
    auto names=tof_names; std::sort(names.begin(),names.end());
    require("tof_names",names==std::vector<std::string>{"front","left","rear","right"},
        "front, rear, left, right, mỗi tên một lần");
    // Hai vòng này kiểm tra an toàn và cập nhật ToF, thời gian nghỉ không quá 100 ms.
    require("control_hz",control_hz>=10,"[10, +inf) Hz (thoi gian nghi kiem tra an toan <= 100 ms)");
    require("sensor_hz",sensor_hz>=10,"[10, +inf) Hz (thoi gian nghi cap nhat ToF <= 100 ms)");
    require("tof_timeout_ms",tof_timeout_ms>=1 && tof_timeout_ms<=500,"[1, 500] ms");
    require("camera_hz",camera_hz>=1,"[1, +inf) Hz");
    require("odometry_hz",odometry_hz>=1,"[1, +inf) Hz");
    require("ui_hz",ui_hz>=1,"[1, +inf) Hz");
    require("servo_trim_deg",servo_trim_deg>=-60 && servo_trim_deg<=60,"[-60, 60] độ");
    require("max_steering_deg",max_steering_deg>=0 && max_steering_deg<=60-std::abs(servo_trim_deg),
        "[0, "+std::to_string(60-std::abs(servo_trim_deg))+"] độ (60 - |servo_trim_deg|)");
    require("tof_brake_mm",tof_brake_mm>=1,"[1, +inf) mm");
    require("imu_drift_ms",imu_drift_ms>=1,"[1, +inf) ms");
    require("image_every_n",image_every_n>=1,"[1, +inf)");
    require("jpeg_quality",jpeg_quality>=1 && jpeg_quality<=100,"[1, 100]");
    require("image_queue_size",image_queue_size>=1,"[1, +inf)");
    require("speed_filter_size",speed_filter_size>=1,"[1, +inf)");
    require("motor_keepalive_ms",motor_keepalive_ms>=1 && motor_keepalive_ms<=200,"[1, 200] ms");
    require("fps_report_ms",fps_report_ms>=1,"[1, +inf) ms");
    require("yaw_axis",yaw_axis>=0 && yaw_axis<=2,"[0, 2]");
    require("pitch_axis",pitch_axis>=0 && pitch_axis<=2,"[0, 2]");
    require("roll_axis",roll_axis>=0 && roll_axis<=2,"[0, 2]");
    require("yaw_sign",std::abs(yaw_sign)==1,"{-1, 1}");
    require("pitch_sign",std::abs(pitch_sign)==1,"{-1, 1}");
    require("roll_sign",std::abs(roll_sign)==1,"{-1, 1}");
    require("keyboard_speed_step_cms",std::isfinite(keyboard_speed_step_cms) && keyboard_speed_step_cms>0,"(0, +inf) cm/s");
    require("keyboard_steering_step_deg",keyboard_steering_step_deg>=1,"[1, +inf) độ");
    for(auto it=fs.root().begin(); it!=fs.root().end(); ++it)
        if(!known_keys.count((*it).name())) throw std::runtime_error("khoá lạ: "+(*it).name());
}
int Config::FrontSlot() const {
    const auto i=std::find(tof_names.begin(),tof_names.end(),"front");
    return i==tof_names.end() ? -1 : static_cast<int>(i-tof_names.begin());
}
int Config::RearSlot() const {
    const auto i=std::find(tof_names.begin(),tof_names.end(),"rear");
    return i==tof_names.end() ? -1 : static_cast<int>(i-tof_names.begin());
}
}
