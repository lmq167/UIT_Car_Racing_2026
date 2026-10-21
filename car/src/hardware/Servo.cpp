#include "hardware/Servo.h"
#include "common/Logger.h"

#include <string>
#include <algorithm>
#include <cerrno>
#include <cstring>

Servo::~Servo()
{
    if (_device != -1)
        close(_device);
}

bool Servo::Init()
{
    ready_ = false;
    _freq = 0;
    if (_device < 0)
        _device = open("/dev/i2c-1", O_RDWR);
    if (_device < 0)
    {
        common::Logger::Error("Servo", "open", std::strerror(errno));
        return false;
    }
    if (ioctl(_device, I2C_SLAVE, _address) != 0)
    {
        common::Logger::Error("Servo", "ioctl", std::strerror(errno));
        return false;
    }
    if (!Reset())
        return false;
    ready_ = true;
    return true;
}

bool Servo::WriteRegister(uint8_t reg, uint8_t value)
{
    if (_device < 0)
        return false;
    if (i2c_smbus_write_byte_data(_device, reg, value) >= 0)
        return true;
    const int error = errno;
    common::Logger::Error("Servo", "i2c_write", std::string(std::strerror(error)) + "; port kept open");
    return false;
}

bool Servo::Reset()
{
    return WriteRegister(MODE1, 0x00 | (1 << 7)) && WriteRegister(MODE2, 0x00 | (1 << 2));
}

bool Servo::SetPWMFreq(uint16_t Freg)
{
    if (_device < 0 || Freg == 0)
        return false;
    _freq = Freg;
    _max_pwm = (uint16_t)(2500.0 / (1000000.0 / ((float)_freq * 4096.0))); //  2500 / (T_set / 4096)
    _min_pwm = (uint16_t)(500.0 / (1000000.0 / ((float)_freq * 4096.0)));
    uint8_t prescale_value = INTERNAL_CLOCK / (4096 * Freg) - 1;
    return WriteRegister(MODE1, 0x00 | (1 << 4)) &&
           WriteRegister(PRE_SCALER, prescale_value) && Reset();
}

bool Servo::SetAngle(int16_t Angle)
{
    if (!ready_ || _freq == 0)
        return false;
    Angle = std::min(MAXANGLE, std::max(Angle, (int16_t)-MAXANGLE)) + 90;
    uint16_t _value = (uint16_t)(_min_pwm + ((Angle * (_max_pwm - _min_pwm)) / 180));
    return SetPWM(15, 0, _value);
}

bool Servo::SetPWM(uint8_t channel, uint16_t On_value, uint16_t Off_value)
{
    uint8_t res_addr = LED_BASE_RES + 4 * channel;
    return WriteRegister(res_addr + 0, On_value & 0xFF) &&
           WriteRegister(res_addr + 1, On_value >> 8) &&
           WriteRegister(res_addr + 2, Off_value & 0xFF) &&
           WriteRegister(res_addr + 3, Off_value >> 8);
}
