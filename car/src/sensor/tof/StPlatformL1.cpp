#include "vl53l1_platform.h"
#include "StTransport.h"
#include <chrono>
#include <thread>

extern "C" {
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t* data, uint32_t count) {
    const int result = tof::stRead(Dev->i2c_slave_address, index, true, data, count);
    return result == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}
VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t* data, uint32_t count) {
    const int result = tof::stWrite(Dev->i2c_slave_address, index, true, data, count);
    return result == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}
VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    uint8_t bytes[1];
    for (unsigned i = 0; i < 1; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (1 - 1 - i)));
    return VL53L1_WriteMulti(Dev, index, bytes, 1);
}
VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t* data) {
    uint8_t bytes[1]{};
    const auto status = VL53L1_ReadMulti(Dev, index, bytes, 1);
    if (status == VL53L1_ERROR_NONE) {
        uint8_t value = 0;
        for (unsigned i = 0; i < 1; ++i) value = static_cast<uint8_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}
VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t bytes[2];
    for (unsigned i = 0; i < 2; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (2 - 1 - i)));
    return VL53L1_WriteMulti(Dev, index, bytes, 2);
}
VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t* data) {
    uint8_t bytes[2]{};
    const auto status = VL53L1_ReadMulti(Dev, index, bytes, 2);
    if (status == VL53L1_ERROR_NONE) {
        uint16_t value = 0;
        for (unsigned i = 0; i < 2; ++i) value = static_cast<uint16_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}
VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    uint8_t bytes[4];
    for (unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (4 - 1 - i)));
    return VL53L1_WriteMulti(Dev, index, bytes, 4);
}
VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t* data) {
    uint8_t bytes[4]{};
    const auto status = VL53L1_ReadMulti(Dev, index, bytes, 4);
    if (status == VL53L1_ERROR_NONE) {
        uint32_t value = 0;
        for (unsigned i = 0; i < 4; ++i) value = static_cast<uint32_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}

VL53L1_Error VL53L1_GetTickCount(VL53L1_DEV, uint32_t* timeMs) {
    *timeMs = tof::monotonicMs();
    return VL53L1_ERROR_NONE;
}
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV, int32_t ms) {
    if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return VL53L1_ERROR_NONE;
}
VL53L1_Error VL53L1_WaitUs(VL53L1_DEV, int32_t us) { tof::sleepUs(us); return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GetTimerFrequency(int32_t* frequency) { *frequency = 1000; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GetTimerValue(int32_t* count) { *count = static_cast<int32_t>(tof::monotonicMs()); return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeoutMs,
    uint16_t index, uint8_t value, uint8_t mask, uint32_t pollDelayMs) {
    const auto start = tof::monotonicMs();
    do {
        uint8_t current = 0;
        const auto status = VL53L1_RdByte(Dev, index, &current);
        if (status != VL53L1_ERROR_NONE) return status;
        if ((current & mask) == value) return VL53L1_ERROR_NONE;
        if (tof::monotonicMs() - start >= timeoutMs) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
    } while (true);
    return VL53L1_ERROR_TIME_OUT;
}
}

