#pragma once
#include <opencv2/core.hpp>
#include "sensor/BNO055.h"
#include "sensor/TOF.h"
#include <chrono>
#include <cstdint>
#include <string>
namespace common {
using Clock = std::chrono::steady_clock;
struct ControlCommand { float speedCmPerSec=0; int steeringDeg=0; bool position_control=false; float position_cm=0; std::uint64_t marker=0; };
struct Observation {
    cv::Mat frame;
    std::uint64_t frame_id=0;
    Clock::time_point captured_at{}, sensor_at{}, odom_at{};
    BNO055::Data imu{};
    sensor::TOFReadings tof{};
    bool imu_valid=false, odom_valid=false;
    double odom_cm=0, meas_speed=0, yaw_rel_deg=0, pitch_deg=0, roll_deg=0;
};
struct Snapshot { Observation observation; ControlCommand requested, applied; std::string state="DUNG",requested_state="LAI_TAY"; double fps=0, actuator_ms=0; Clock::time_point sampled_at{}; };
}
