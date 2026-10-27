#pragma once
#include <string>
#include <vector>
namespace common {
struct Config {
    std::string loaded_text;
    std::string camera_pipeline = "nvarguscamerasrc sensor-mode=5 wbmode=0 ! video/x-raw(memory:NVMM),width=1080,height=720,framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,width=640,height=384,format=BGR ! appsink drop=true max-buffers=1";
    std::string fake_source = "";
    int camera_hz = 50;
    int sensor_hz = 50;
    int control_hz = 50;
    int odometry_hz = 20;
    int ui_hz = 5;
    float max_speed_cms = 25.0f;
    int max_steering_deg = 20;
    int servo_trim_deg = 0;
    int servo_invert = 0;
    int tof_safety_enabled = 1;
    int tof_brake_mm = 250;
    int tof_timeout_ms = 500;
    int image_timeout_ms = 500;
    float camera_resume_max_s = 2.0f;
    float resume_stable_s = 2.0f;
    float soft_start_s = 1.0f;
    int tof_mapping_confirmed = 0;
    int camera_width = 640, camera_height = 384;
    float smoke_min_fps = 10.0f;
    float smoke_stop_speed_cms = 1.0f;
    int smoke_servo_deg = 10;
    float calib_speed_cms = 10.0f;
    int calib_servo_step_deg = 5;
    int calib_servo_limit_deg = 30;
    float calib_distance_cm = 200.0f;
    float calib_position_cm = 50.0f;
    float calib_max_duration_s = 60.0f;
    std::string imu_calibration_file = "../include/sensor/bno055_calibration.txt";
    int imu_drift_ms = 500;
    int yaw_axis = 2;
    int pitch_axis = 1;
    int roll_axis = 2;
    float yaw_sign = 1.0f;
    float pitch_sign = 1.0f;
    float roll_sign = 1.0f;
    std::string log_dir = "../runs";
    int image_every_n = 5;
    int jpeg_quality = 85;
    int image_queue_size = 8;
    int speed_filter_size = 5;
    int enable_oled = 1;
    int enable_display = 1;
    float keyboard_speed_step_cms = 5.0f;
    int keyboard_steering_step_deg = 5;
    int motor_keepalive_ms = 200;
    int fps_report_ms = 2000;
    int collect_fps = 3;
    float collect_min_speed_cms = 1.0f;
    float collect_duplicate_threshold = 0.8f;
    int collect_confirm_ms = 2000;
    // Thứ tự: nhanh, chậm, trái, phải, dừng, ghi, xóa, thoát, mục tiêu -/+, nhịp +/-.
    std::vector<std::string> collect_keys{"w","s","a","d"," ","c","u","esc","[","]","+","-"};
    std::string collect_luot_keys = "123456789";
    std::vector<std::string> tof_names{"front", "rear", "left", "right"};
    void Load(const std::string &path);
    int FrontSlot() const;
    int RearSlot() const;
};
}
