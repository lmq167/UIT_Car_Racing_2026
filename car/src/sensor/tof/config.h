#pragma once
#include <array>
#include <cstdint>

namespace tof {
// Jetson Nano (Tegra210), standard 40-pin header. NOT Orin Nano.
struct Slot { int boardPin; int gpioOffset; uint8_t address; };
constexpr std::array<Slot, 4> kSlots{{
    {32, 168, 0x32}, {36, 51, 0x33}, {38, 77, 0x30}, {40, 78, 0x31}
}};
// All addresses in this Linux project are 7-bit.
constexpr uint8_t kDefaultAddress = 0x29;
constexpr const char* kI2cDevice = "/dev/i2c-1";
// Cooperative deadline for each ST initialization/start operation.
constexpr unsigned kInitStepTimeoutMs = 10000;
}
