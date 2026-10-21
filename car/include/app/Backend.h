#pragma once
#include "common/Config.h"
#include "common/Types.h"
#include <opencv2/videoio.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#ifndef UCR_FAKE_HW
#include "hardware/Motor.h"
#include "hardware/Servo.h"
#include "hardware/SH1106.h"
#endif
namespace app {
class Backend {
public:
    explicit Backend(const common::Config &config);
    ~Backend();
    void Init();
    bool EmergencyOpenAndStop();
    bool Frame(cv::Mat &frame);
    bool Sensors(common::Observation &o, double dt);
    bool Position(float &cm);
    void Apply(common::ControlCommand c, bool send_motor=true);
    void Stop() noexcept;
    void Render(const common::Snapshot &s, const std::string &mode);
private:
    common::Config c_;
    cv::VideoCapture camera_;
    std::mutex motor_mutex_, i2c_mutex_;
    bool motor_ready_=false, servo_ready_=false, oled_ready_=false;
    double yaw_=0, bias_=0;
    std::atomic<bool> stopping_{false};
#ifndef UCR_FAKE_HW
    Motor motor_;
    Servo servo_;
    SH1106 oled_;
    BNO055 imu_;
    sensor::TOF tof_;
#endif
};
}
