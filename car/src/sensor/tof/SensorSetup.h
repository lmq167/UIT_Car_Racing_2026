#pragma once
#include "config.h"
#include <iosfwd>
#include <cstddef>

namespace tof {
enum class SensorType { Unknown, L0, L1 };
const char* typeName(SensorType type);

// Transport interface also permits testing without physical sensors.
class Hardware {
public:
    virtual ~Hardware() = default;
    virtual void shutdown(std::size_t slot, bool enabled) = 0;
    virtual void delayMs(unsigned ms) = 0;
    virtual uint8_t readByte(uint8_t address, uint16_t reg, bool wideRegister) = 0;
    virtual void writeByte(uint8_t address, uint16_t reg, bool wideRegister, uint8_t value) = 0;
    virtual void readBlock(uint8_t address, uint16_t reg, bool wideRegister, uint8_t* data, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i)
            data[i] = readByte(address, static_cast<uint16_t>(reg + i), wideRegister);
    }
    virtual void writeBlock(uint8_t address, uint16_t reg, bool wideRegister, const uint8_t* data, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i)
            writeByte(address, static_cast<uint16_t>(reg + i), wideRegister, data[i]);
    }
};

class SensorSetup {
public:
    explicit SensorSetup(Hardware& hardware) : hardware_(hardware) {}
    // True when at least one slot is assigned; failed slots remain Unknown.
    bool run(std::ostream& log);
    const std::array<SensorType, 4>& types() const { return types_; }
private:
    std::array<SensorType, 4> types_{};
    Hardware& hardware_;
    SensorType detect(uint8_t address);
    bool verify(SensorType type, uint8_t address);
};
}
