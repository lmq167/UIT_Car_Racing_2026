#pragma once
#include <opencv2/core.hpp>
#include "sensor/BNO055.h"
#include "sensor/TOF.h"
#include <chrono>
#include <cstdint>
#include <string>
namespace common {
using Clock = std::chrono::steady_clock;
// Mã trạng thái truyền về giao diện BTC, giữ nguyên thứ tự 0–13.
enum class VehicleState : int32_t {
    NONE = 0,
    GREEN_LIGHT = 1,
    RED_LIGHT = 2,
    ROAD_WORK = 3,
    BRIDGE = 4,
    ROUNDABOUT = 5,
    OBSTACLE = 6,
    TURN_RIGHT = 7,
    NO_TURN_RIGHT = 8,
    TURN_LEFT = 9,
    NO_TURN_LEFT = 10,
    PARKING_SIGN = 11,
    PARKING = 12,
    PEDESTRIAN = 13
};
struct CalibrationLimits {
    bool enabled=false;
    Clock::time_point deadline{};
    double start_odom_cm=0, max_distance_cm=0;
    bool limit_yaw=false;
    double start_yaw_deg=0, max_yaw_deg=360;
};
struct ControlCommand {
    float speedCmPerSec=0;
    int steeringDeg=0;
    bool position_control=false;
    float position_cm=0;
    std::uint64_t marker=0;
    CalibrationLimits calib_limits{};
};
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
