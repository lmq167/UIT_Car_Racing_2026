#ifndef SERVO_H_
#define SERVO_H_
#include <linux/i2c-dev.h>
#include <cstdint>
#include <atomic>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
// #include <cmath>

extern "C" 
{
    #include <i2c/smbus.h>
}

#define MODE1 0x00
#define MODE2 0x01
#define INTERNAL_CLOCK 25 * 1000 * 1000
#define LED_BASE_RES 0x06
#define PRE_SCALER 0xFE
#define MAXANGLE (int16_t)60


class Servo 
{
    private:
        uint8_t _address;
        int _device;
        uint16_t _freq{0};
        uint16_t _max_pwm{0}, _min_pwm{0};
        std::atomic<bool> ready_{false};
        bool WriteRegister(uint8_t reg, uint8_t value);
    public:
        Servo(uint8_t address = 0x40) : _address(address), _device(-1){}
        ~Servo();
        bool Init();
        bool IsReady() const { return ready_.load(); }
        bool SetPWMFreq(uint16_t Freq);
        bool Reset();
        bool SetAngle(int16_t Angle);
        bool SetPWM(uint8_t channel, uint16_t On_value, uint16_t Off_value);

};


#endif
