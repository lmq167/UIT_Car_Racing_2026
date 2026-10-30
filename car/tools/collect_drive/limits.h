#pragma once
#include <cmath>

namespace collect_drive {
constexpr float kSpeedStepCms = 5.0f;
constexpr float kMaxSpeedCms = 40.0f;
constexpr float kHwSpeedCms = 60.0f;
constexpr int kSteerStepDeg = 5;
constexpr int kMaxSteerDeg = 40;
constexpr int kHwSteerDeg = 60;
constexpr bool kReverseMotor = false;
constexpr int kDefaultIntervalMs = 1000;
constexpr int kStaleFrameMs = 500;
constexpr int kPollMs = 10;
constexpr int kUartPollMs = 50;
constexpr int kHardwareTimeoutMs = 300;
constexpr int kInitTimeoutMs = 5000;
constexpr int kFrameTimeoutMs = 5000;
constexpr int kIoTimeoutMs = 5000;
constexpr int kCleanupTimeoutMs = 3000;
constexpr int kStopTimeoutMs = 10000;
constexpr int kStopKillMs = 1000;
constexpr int kDiskCheckMs = 1000;
constexpr int kMinFreeMb = 1024;
constexpr int kJpegQuality = 90;
constexpr int kServoPwmHz = 50;
constexpr unsigned long kMaxFrameBytes = 64UL * 1024 * 1024;
constexpr const char* kPipeline =
    "nvarguscamerasrc sensor-mode=5 wbmode=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1";

static_assert(kSpeedStepCms > 0 && kSpeedStepCms <= kMaxSpeedCms && kMaxSpeedCms <= kHwSpeedCms,
              "Buoc <= tran phim <= tran phan cung");
static_assert(kSteerStepDeg > 0 && kSteerStepDeg <= kMaxSteerDeg && kMaxSteerDeg <= kHwSteerDeg,
              "Buoc <= tran phim <= tran phan cung");
inline float clampKeySpeed(float v) {
    if (!std::isfinite(v)) return 0.0f;
    return v < -kMaxSpeedCms ? -kMaxSpeedCms : v > kMaxSpeedCms ? kMaxSpeedCms : v;
}
inline float clampHwSpeed(float v) {
    if (!std::isfinite(v)) return 0.0f;
    return v < -kHwSpeedCms ? -kHwSpeedCms : v > kHwSpeedCms ? kHwSpeedCms : v;
}
inline int clampKeySteer(int a) {
    return a < -kMaxSteerDeg ? -kMaxSteerDeg : a > kMaxSteerDeg ? kMaxSteerDeg : a;
}
inline int clampHwSteer(int a) {
    return a < -kHwSteerDeg ? -kHwSteerDeg : a > kHwSteerDeg ? kHwSteerDeg : a;
}
inline float applyMotorSign(float v, bool reverse) {
    return v == 0 ? +0.0f : (reverse ? -v : v);
}
} // namespace collect_drive
