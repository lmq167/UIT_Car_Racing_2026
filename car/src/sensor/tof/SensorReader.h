#pragma once
#include "SensorSetup.h"
#include <memory>

namespace tof {
struct Measurement {
    bool ready = false;
    int apiError = 0;
    uint8_t rangeStatus = 255;
    int distanceMm = 0;
};
class SensorReader {
public:
    SensorReader(Hardware& hardware, const std::array<SensorType, 4>& types);
    ~SensorReader();
    // Attempts every identified slot; throws only if none starts successfully.
    void start(std::ostream& log);
    bool initialized(std::size_t slot) const;
    Measurement read(std::size_t slot);
    bool stop(std::ostream& log);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
