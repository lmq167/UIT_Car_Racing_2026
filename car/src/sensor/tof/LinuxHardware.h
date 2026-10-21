#pragma once
#include "SensorSetup.h"
#include <string>

namespace tof {
class LinuxHardware final : public Hardware {
public:
    explicit LinuxHardware(const std::string& device);
    ~LinuxHardware() override;
    LinuxHardware(const LinuxHardware&) = delete;
    LinuxHardware& operator=(const LinuxHardware&) = delete;
    void shutdown(std::size_t slot, bool enabled) override;
    void delayMs(unsigned ms) override;
    uint8_t readByte(uint8_t address, uint16_t reg, bool wideRegister) override;
    void writeByte(uint8_t address, uint16_t reg, bool wideRegister, uint8_t value) override;
    void readBlock(uint8_t address, uint16_t reg, bool wideRegister, uint8_t* data, std::size_t count) override;
    void writeBlock(uint8_t address, uint16_t reg, bool wideRegister, const uint8_t* data, std::size_t count) override;
private:
    int fd_ = -1;
    int gpioBase_ = 0;
    std::array<bool, 4> prepared_{{false, false, false, false}};
};
}
