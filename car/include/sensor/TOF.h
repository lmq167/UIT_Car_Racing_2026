#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace sensor
{
    struct TOFReading
    {
        int boardPin{0};
        std::uint8_t address{0}; // Linux 7-bit I2C address
        int distanceMm{-1};    // -1 when no valid, current distance is available
        std::uint8_t rangeStatus{255};
        int apiError{0};
        bool hasSample{false};
        bool valid{false};
        bool timedOut{false};
        bool initialized{false};
        std::chrono::steady_clock::time_point updatedAt{};
    };

    using TOFReadings = std::array<TOFReading, 4>;

    // Owns the four-device cluster; callers serialize access to the I2C bus.
    class TOF
    {
    public:
        TOF();
        ~TOF();
        TOF(const TOF &) = delete;
        TOF &operator=(const TOF &) = delete;

        bool Init(const std::string &device);
        bool Read(TOFReadings &readings, std::chrono::milliseconds timeout);
        bool Stop();
        const std::string &LastError() const { return lastError_; }

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        std::string lastError_;
    };
} // namespace sensor
