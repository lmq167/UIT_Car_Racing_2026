#pragma once
#include "control/Planner.h"
#include <cmath>

namespace auto_drive {
constexpr float kMaxAutoSpeedCms = 20;
constexpr int kMaxAutoSteerDeg = 40;
// Riêng pha đỗ (Planner ở DO_XE; Quân 11/11 18:54): lái tới 45°. Chặng khác giữ 40°.
constexpr int kMaxParkSteerDeg = 45;
constexpr bool kReverseMotor = false;
constexpr int kStaleFrameMs = control::kStaleImageMs, kPollMs = 10, kUartPollMs = 50;
constexpr int kHardwareTimeoutMs = 300, kInitTimeoutMs = 5000;
constexpr int kFrameTimeoutMs = 5000, kCleanupTimeoutMs = 3000;
constexpr int kStopTimeoutMs = 10000, kStopKillMs = 1000;
constexpr int kServoPwmHz = 50, kMinFreeMb = 1024, kJpegQuality = 90;
constexpr unsigned long kMaxFrameBytes = 64UL * 1024 * 1024;
constexpr const char* kPipeline =
    "nvarguscamerasrc sensor-mode=5 wbmode=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
inline float clampAutoSpeed(float v) {
    return !std::isfinite(v) ? 0 : v < -kMaxAutoSpeedCms ? -kMaxAutoSpeedCms :
        v > kMaxAutoSpeedCms ? kMaxAutoSpeedCms : v;
}
inline int clampAutoSteer(int a, bool park = false) {
    const int m = park ? kMaxParkSteerDeg : kMaxAutoSteerDeg;
    return a < -m ? -m : a > m ? m : a;
}
inline float applyMotorSign(float v, bool reverse) { return v == 0 ? +0.0f : reverse ? -v : v; }
}
