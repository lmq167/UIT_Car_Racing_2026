#ifndef UCR_DRIVE_STUB_I2C_SMBUS_H
#define UCR_DRIVE_STUB_I2C_SMBUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t i2c_smbus_write_byte_data(int file, uint8_t command, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
