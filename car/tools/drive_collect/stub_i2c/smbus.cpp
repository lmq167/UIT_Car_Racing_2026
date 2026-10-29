#include "i2c/smbus.h"

#include <cerrno>

// Không truy cập thiết bị và không báo thành công giả khi kiểm thử.
int32_t i2c_smbus_write_byte_data(int, uint8_t, uint8_t)
{
    errno = ENOSYS;
    return -1;
}
