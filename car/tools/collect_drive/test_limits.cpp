// T33_contract.md: kiểm giới hạn độc lập, kể cả khi build Release có NDEBUG.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "limits.h"

using namespace collect_drive;
static_assert(kSpeedStepCms == 5.0f && kMaxSpeedCms == 40.0f && kHwSpeedCms == 60.0f,
              "T33: hang toc do");
static_assert(kSteerStepDeg == 5 && kMaxSteerDeg == 40 && kHwSteerDeg == 60,
              "T33: hang goc");
static_assert(kSpeedStepCms <= kMaxSpeedCms && kMaxSpeedCms <= kHwSpeedCms,
              "T33: thu tu gioi han toc do");
static_assert(kSteerStepDeg <= kMaxSteerDeg && kMaxSteerDeg <= kHwSteerDeg,
              "T33: thu tu gioi han goc");
static_assert(!kReverseMotor && kDefaultIntervalMs == 1000 && kStaleFrameMs == 500,
              "T33: mac dinh");

static void check(bool ok, const char* tag) {
    if (!ok) std::cerr << tag << '\n';
    assert(ok);
}

int main() {
    // Kỳ vọng là số cố định từ hợp đồng, không lấy trần từ hàm đang kiểm.
    for (float v : {-1000.0f, -61.0f, -60.0f, -59.0f, -41.0f, -40.0f,
                    -39.0f, -5.0f, -0.0f, 0.0f, 5.0f, 39.0f, 40.0f,
                    41.0f, 59.0f, 60.0f, 61.0f, 1000.0f}) {
        const float key = v < -40 ? -40 : (v > 40 ? 40 : v);
        const float hw = v < -60 ? -60 : (v > 60 ? 60 : v);
        check(clampKeySpeed(v) == key, "limits_key_speed");
        check(clampHwSpeed(v) == hw, "limits_hw_speed");
        for (bool reverse : {false, true}) {
            const float got = applyMotorSign(v, reverse);
            check(got == (v == 0 ? 0 : (reverse ? -v : v)), "limits_sign");
            check(v != 0 || !std::signbit(got), "limits_positive_zero");
        }
    }
    for (int a : {std::numeric_limits<int>::min(), -1000, -61, -60, -59,
                  -41, -40, -39, -5, 0, 5, 39, 40, 41, 59, 60, 61, 1000,
                  std::numeric_limits<int>::max()}) {
        check(clampKeySteer(a) == (a < -40 ? -40 : (a > 40 ? 40 : a)),
              "limits_key_steer");
        check(clampHwSteer(a) == (a < -60 ? -60 : (a > 60 ? 60 : a)),
              "limits_hw_steer");
    }
    std::cout << "OK limits T33\n";
}
