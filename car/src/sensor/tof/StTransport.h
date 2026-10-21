#pragma once
#include "SensorSetup.h"
#include <cstdint>
#include <string>

namespace tof {
// ST APIs use 8-bit device addresses; convert at this transport boundary.
void setStHardware(Hardware* hardware);
const std::string& stLastError();
void stSetDeadlineMs(unsigned timeoutMs);
void stClearDeadline();
bool stDeadlineExpired();
int stRead(uint8_t address8, uint16_t reg, bool wide, uint8_t* data, uint32_t count) noexcept;
int stWrite(uint8_t address8, uint16_t reg, bool wide, const uint8_t* data, uint32_t count) noexcept;
uint32_t monotonicMs();
void sleepUs(int32_t us);
}
