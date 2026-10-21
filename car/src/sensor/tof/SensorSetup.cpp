#include "SensorSetup.h"
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace tof {
namespace {
std::string hexAddress(uint8_t address) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << std::setw(2)
        << std::setfill('0') << static_cast<unsigned>(address);
    return out.str();
}
}
const char* typeName(SensorType type) {
    switch (type) {
    case SensorType::L0: return "VL53L0X";
    case SensorType::L1: return "VL53L1X";
    default: return "UNKNOWN";
    }
}

bool SensorSetup::verify(SensorType type, uint8_t address) {
    if (type == SensorType::L0)
        return hardware_.readByte(address, 0xC0, false) == 0xEE;
    if (type == SensorType::L1)
        return hardware_.readByte(address, 0x010F, true) == 0xEA;
    return false;
}

SensorType SensorSetup::detect(uint8_t address) {
    // Test L0 first: its register pointer is one byte. A two-byte L1
    // pointer would be interpreted as a register write by an L0 sensor.
    try { if (verify(SensorType::L0, address)) return SensorType::L0; }
    catch (const std::exception&) { /* L1 may reject a one-byte pointer. */ }
    try { if (verify(SensorType::L1, address)) return SensorType::L1; }
    catch (const std::exception&) { /* Not ready, disconnected, or unknown. */ }
    return SensorType::Unknown;
}

bool SensorSetup::run(std::ostream& log) {
    types_.fill(SensorType::Unknown);
    auto& types = types_;
    std::size_t current = 0;
    try {
        // Reset every device before talking to the shared default address.
        for (current = 0; current < kSlots.size(); ++current)
            hardware_.shutdown(current, false);
        hardware_.delayMs(20);
        for (current = 0; current < kSlots.size(); ++current) {
            try {
                const auto& slot = kSlots[current];
                hardware_.shutdown(current, true);
                hardware_.delayMs(150);
                SensorType type = SensorType::Unknown;
                for (unsigned attempt = 0; attempt < 10; ++attempt) {
                    type = detect(kDefaultAddress);
                    if (type != SensorType::Unknown) break;
                    hardware_.delayMs(20);
                }
                if (type == SensorType::Unknown)
                    throw std::runtime_error("No supported L0/L1 ID at 0x29; check XSHUT, wiring and pinmux");
                // Same address registers used by the ST APIs in the ESP32 project.
                // Linux passes the 7-bit address directly, without shifting.
                hardware_.writeByte(kDefaultAddress,
                    type == SensorType::L0 ? 0x008A : 0x0001,
                    type == SensorType::L1, slot.address);
                hardware_.delayMs(5);
                if (!verify(type, slot.address))
                    throw std::runtime_error("Model ID verification failed at new address");
                types[current] = type;
                log << "[SET] XSHUT BOARD " << slot.boardPin << " -> " << typeName(type)
                    << " I2C(7-bit)=" << hexAddress(slot.address)
                    << " ST(8-bit)=" << hexAddress(static_cast<uint8_t>(slot.address << 1)) << '\n';
            } catch (const std::exception& error) {
                types[current] = SensorType::Unknown;
                log << "[ERROR] XSHUT BOARD " << kSlots[current].boardPin << ": " << error.what() << '\n';
                // Isolate an unassigned device before enabling the next at 0x29.
                hardware_.shutdown(current, false);
            }
        }
        // Verify all four again while every sensor is enabled.
        for (current = 0; current < kSlots.size(); ++current) {
            if (types[current] == SensorType::Unknown) continue;
            try {
                if (!verify(types[current], kSlots[current].address))
                    throw std::runtime_error("Final verification failed with all sensors enabled");
            } catch (const std::exception& error) {
                types[current] = SensorType::Unknown;
                log << "[ERROR] XSHUT BOARD " << kSlots[current].boardPin << ": " << error.what() << '\n';
                hardware_.shutdown(current, false);
            }
        }
    } catch (const std::exception& error) {
        log << "[ERROR]";
        if (current < kSlots.size()) log << " XSHUT BOARD " << kSlots[current].boardPin;
        log << ": " << error.what() << '\n';
        // Do not continue with an unassigned device at the shared default address.
        for (std::size_t i = 0; i < kSlots.size(); ++i) {
            try { hardware_.shutdown(i, false); }
            catch (const std::exception& cleanup) {
                log << "[ERROR] Cannot pull BOARD " << kSlots[i].boardPin << " LOW: " << cleanup.what() << '\n';
            }
        }
        log << "Setup FAILED; no successful address summary.\n";
        types_.fill(SensorType::Unknown);
        return false;
    }
    unsigned verified = 0;
    for (const auto type : types) if (type != SensorType::Unknown) ++verified;
    log << "\nSetup: " << verified << "/4 sensors verified.\n";
    for (const auto type : {SensorType::L0, SensorType::L1}) {
        unsigned count = 0;
        for (const auto found : types) if (found == type) ++count;
        log << typeName(type) << ": " << count << " sensor(s); addresses:";
        for (std::size_t i = 0; i < types.size(); ++i)
            if (types[i] == type) log << ' ' << hexAddress(kSlots[i].address);
        log << '\n';
    }
    return verified != 0;
}
}
