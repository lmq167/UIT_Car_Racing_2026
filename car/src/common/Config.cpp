#include "common/Config.h"
#include "common/ConfigValue.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace common {
void Config::Load(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Khong mo duoc config: " + path);
    auto read = [&](const char *name, auto &value) {
        if (!HasConfigKey(fs.root(),name)) std::cerr << "Thieu khoa, dung mac dinh: " << name << '\n';
        else ReadConfigValue(name,fs[name],value);
    };
    auto flag = [&](const char *name, int &value) {
        const auto node=fs[name];
        if(!HasConfigKey(fs.root(),name)) { std::cerr << "Thieu khoa, dung mac dinh: " << name << '\n'; return; }
        ReadConfigFlag(name,node,value);
    };
    read("camera_pipeline", camera_pipeline);
    read("fake_source", fake_source);
    read("camera_hz", camera_hz);
    read("sensor_hz", sensor_hz);
    read("control_hz", control_hz);
    read("odometry_hz", odometry_hz);
    read("ui_hz", ui_hz);
    read("max_speed_cms", max_speed_cms);
    read("max_steering_deg", max_steering_deg);
    read("servo_trim_deg", servo_trim_deg);
    flag("servo_invert", servo_invert);
    flag("tof_safety_enabled", tof_safety_enabled);
    read("tof_brake_mm", tof_brake_mm);
    read("tof_timeout_ms", tof_timeout_ms);
    read("image_timeout_ms", image_timeout_ms);
    read("imu_calibration_file", imu_calibration_file);
    read("imu_drift_ms", imu_drift_ms);
    read("yaw_axis", yaw_axis);
    read("pitch_axis", pitch_axis);
    read("roll_axis", roll_axis);
    read("yaw_sign", yaw_sign);
    read("pitch_sign", pitch_sign);
    read("roll_sign", roll_sign);
    read("log_dir", log_dir);
    read("image_every_n", image_every_n);
    read("jpeg_quality", jpeg_quality);
    read("image_queue_size", image_queue_size);
    read("speed_filter_size", speed_filter_size);
    flag("enable_oled", enable_oled);
    flag("enable_display", enable_display);
    read("keyboard_speed_step_cms", keyboard_speed_step_cms);
    read("keyboard_steering_step_deg", keyboard_steering_step_deg);
    read("motor_keepalive_ms", motor_keepalive_ms);
    read("fps_report_ms", fps_report_ms);
    read("collect_fps", collect_fps);
    read("collect_min_speed_cms", collect_min_speed_cms);
    read("collect_duplicate_threshold", collect_duplicate_threshold);
    read("collect_confirm_ms", collect_confirm_ms);
    read("collect_keys", collect_keys);
    read("collect_luot_keys", collect_luot_keys);
    if(collect_fps<1 || collect_fps>10 || !std::isfinite(collect_min_speed_cms) || collect_min_speed_cms<=0 ||
       !std::isfinite(collect_duplicate_threshold) || collect_duplicate_threshold<0 || collect_duplicate_threshold>255 ||
       collect_confirm_ms<1 || collect_keys.size()!=12 || collect_luot_keys.size()!=9)
        throw std::runtime_error("Cấu hình thu ảnh không hợp lệ: nhịp 1–10, tốc độ tối thiểu > 0, ngưỡng trùng 0–255");
    std::string used;
    auto add_key=[&](const std::string &key) {
        const char ch=key=="esc" ? 27 : (key.size()==1 ? key[0] : 0);
        if(!ch || (ch!=27 && (ch<32 || ch>126 || (ch>='A'&&ch<='Z'))) || used.find(ch)!=std::string::npos)
            throw std::runtime_error("Bảng phím thu ảnh phải dùng ký tự ASCII thường, không trùng nhau");
        used+=ch;
    };
    for(const auto &key:collect_keys) add_key(key);
    for(char ch:collect_luot_keys) add_key(std::string(1,ch));
    read("tof_names", tof_names);
    if (tof_names.size()!=4 || FrontSlot()<0) throw std::runtime_error("tof_names phai co 4 slot va mot front");
    auto names=tof_names; std::sort(names.begin(),names.end());
    if (names!=std::vector<std::string>{"front","left","rear","right"}) throw std::runtime_error("Ten ToF khong hop le");
    // Hai vòng này kiểm tra an toàn và cập nhật ToF, thời gian nghỉ không quá 100 ms.
    if (control_hz<10)
        throw std::runtime_error("control_hz phai >= 10 Hz (thoi gian nghi kiem tra an toan <= 100 ms)");
    if (sensor_hz<10)
        throw std::runtime_error("sensor_hz phai >= 10 Hz (thoi gian nghi cap nhat ToF <= 100 ms)");
    if (tof_timeout_ms<1 || tof_timeout_ms>500)
        throw std::runtime_error("tof_timeout_ms="+std::to_string(tof_timeout_ms)+" ngoài khoảng hợp lệ [1, 500] ms");
    if (camera_hz<1 || odometry_hz<1 || ui_hz<1 ||
        !std::isfinite(max_speed_cms) || max_speed_cms<=0 || max_steering_deg<0 ||
        (servo_trim_deg < -60 || servo_trim_deg > 60) || max_steering_deg>60-std::abs(servo_trim_deg) ||
        image_timeout_ms<1 || image_timeout_ms>500 || tof_brake_mm<1 ||
        imu_drift_ms<1 || image_every_n<1 || jpeg_quality<1 || jpeg_quality>100 ||
        image_queue_size<1 || speed_filter_size<1 || motor_keepalive_ms<1 || motor_keepalive_ms>200 || fps_report_ms<1 ||
        yaw_axis<0 || yaw_axis>2 || pitch_axis<0 || pitch_axis>2 || roll_axis<0 || roll_axis>2 ||
        !std::isfinite(yaw_sign) || !std::isfinite(pitch_sign) || !std::isfinite(roll_sign) ||
        std::abs(yaw_sign)!=1 || std::abs(pitch_sign)!=1 || std::abs(roll_sign)!=1 ||
        !std::isfinite(keyboard_speed_step_cms) || keyboard_speed_step_cms<=0 || keyboard_steering_step_deg<1)
        throw std::runtime_error("Gia tri config vuot gioi han an toan");
}
int Config::FrontSlot() const {
    const auto i=std::find(tof_names.begin(),tof_names.end(),"front");
    return i==tof_names.end() ? -1 : static_cast<int>(i-tof_names.begin());
}
}
