#include "vl53l0x_platform.h"
#include "StTransport.h"

extern "C" {
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* data, uint32_t count) {
    const int result = tof::stRead(Dev->I2cDevAddr, index, false, data, count);
    return result == 0 ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* data, uint32_t count) {
    const int result = tof::stWrite(Dev->I2cDevAddr, index, false, data, count);
    return result == 0 ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    uint8_t bytes[1];
    for (unsigned i = 0; i < 1; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (1 - 1 - i)));
    return VL53L0X_WriteMulti(Dev, index, bytes, 1);
}
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t* data) {
    uint8_t bytes[1]{};
    const auto status = VL53L0X_ReadMulti(Dev, index, bytes, 1);
    if (status == VL53L0X_ERROR_NONE) {
        uint8_t value = 0;
        for (unsigned i = 0; i < 1; ++i) value = static_cast<uint8_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t bytes[2];
    for (unsigned i = 0; i < 2; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (2 - 1 - i)));
    return VL53L0X_WriteMulti(Dev, index, bytes, 2);
}
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t* data) {
    uint8_t bytes[2]{};
    const auto status = VL53L0X_ReadMulti(Dev, index, bytes, 2);
    if (status == VL53L0X_ERROR_NONE) {
        uint16_t value = 0;
        for (unsigned i = 0; i < 2; ++i) value = static_cast<uint16_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t bytes[4];
    for (unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<uint8_t>(data >> (8 * (4 - 1 - i)));
    return VL53L0X_WriteMulti(Dev, index, bytes, 4);
}
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t* data) {
    uint8_t bytes[4]{};
    const auto status = VL53L0X_ReadMulti(Dev, index, bytes, 4);
    if (status == VL53L0X_ERROR_NONE) {
        uint32_t value = 0;
        for (unsigned i = 0; i < 4; ++i) value = static_cast<uint32_t>((value << 8) | bytes[i]);
        *data = value;
    }
    return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t andData, uint8_t orData) {
    uint8_t value = 0;
    const auto status = VL53L0X_RdByte(Dev, index, &value);
    if (status != VL53L0X_ERROR_NONE) return status;
    return VL53L0X_WrByte(Dev, index, (value & andData) | orData);
}
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV) { tof::sleepUs(2000); return VL53L0X_ERROR_NONE; }
// The application accesses the bus on a single thread.
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV) { return VL53L0X_ERROR_NONE; }
VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV) { return VL53L0X_ERROR_NONE; }
}

