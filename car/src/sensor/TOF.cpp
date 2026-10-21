#include "sensor/TOF.h"

#include "tof/LinuxHardware.h"
#include "tof/SensorReader.h"

#include <exception>
#include <iostream>
#include <sstream>
#include <utility>

namespace sensor
{
    struct TOF::Impl
    {
        // Reader must be destroyed before the hardware it references.
        std::unique_ptr<tof::LinuxHardware> hardware;
        std::unique_ptr<tof::SensorReader> reader;
        TOFReadings readings{};
        std::chrono::steady_clock::time_point startedAt{};
        bool initialized{false};
    };

    TOF::TOF() = default;
    TOF::~TOF() = default;

    bool TOF::Init(const std::string &device)
    {
        if (impl_ && impl_->initialized)
        {
            lastError_ = "TOF already initialized";
            return false;
        }

        lastError_.clear();
        try
        {
            if (!impl_) impl_ = std::make_unique<Impl>();
            auto *state = impl_.get();
            if (!state->hardware) state->hardware = std::make_unique<tof::LinuxHardware>(device);
            // A previous start may have configured only part of the cluster.
            // Rebuild the ranging state for setup, retaining the hardware port.
            state->reader.reset();
            tof::SensorSetup setup(*state->hardware);
            if (!setup.run(std::cout))
            {
                lastError_ = "no TOF sensor could be assigned an address";
                return false;
            }
            state->reader = std::make_unique<tof::SensorReader>(*state->hardware, setup.types());
            state->reader->start(std::cout);
            state->startedAt = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < state->readings.size(); ++i)
            {
                state->readings[i].boardPin = tof::kSlots[i].boardPin;
                state->readings[i].address = tof::kSlots[i].address;
                state->readings[i].initialized = state->reader->initialized(i);
            }
            state->initialized = true;
            return true;
        }
        catch (const std::exception &ex)
        {
            lastError_ = ex.what();
            return false;
        }
    }

    bool TOF::Read(TOFReadings &readings, std::chrono::milliseconds timeout)
    {
        lastError_.clear();
        if (!impl_ || !impl_->initialized)
        {
            readings = {};
            lastError_ = "TOF is not initialized";
            return false;
        }

        bool ok = true;
        // Poll every slot even if one reports an API error.
        for (std::size_t i = 0; i < impl_->readings.size(); ++i)
        {
            auto &reading = impl_->readings[i];
            if (!reading.initialized) continue;
            try
            {
                const auto sample = impl_->reader->read(i);
                if (sample.ready || sample.apiError != 0)
                {
                    reading.apiError = sample.apiError;
                    reading.rangeStatus = sample.rangeStatus;
                    reading.hasSample = sample.ready;
                    reading.valid = sample.ready && sample.apiError == 0 && sample.rangeStatus == 0;
                    reading.distanceMm = reading.valid ? sample.distanceMm : -1;
                    reading.updatedAt = std::chrono::steady_clock::now();
                }
                if (sample.apiError != 0)
                {
                    ok = false;
                    lastError_ += "BOARD " + std::to_string(reading.boardPin) +
                                  " ST error=" + std::to_string(sample.apiError) + "; ";
                }
            }
            catch (const std::exception &ex)
            {
                reading.valid = false;
                reading.distanceMm = -1;
                reading.apiError = -1;
                ok = false;
                lastError_ += "BOARD " + std::to_string(reading.boardPin) + ": " + ex.what() + "; ";
            }

            const auto since = reading.updatedAt == std::chrono::steady_clock::time_point{}
                                   ? impl_->startedAt : reading.updatedAt;
            reading.timedOut = std::chrono::steady_clock::now() - since > timeout;
            if (reading.timedOut)
            {
                reading.valid = false;
                reading.distanceMm = -1;
            }
        }
        readings = impl_->readings;
        return ok;
    }

    bool TOF::Stop()
    {
        if (!impl_ || !impl_->reader)
        {
            return true;
        }
        std::ostringstream log;
        const bool ok = impl_->reader->stop(log);
        if (!ok)
        {
            lastError_ = log.str();
        }
        impl_.reset();
        return ok;
    }
} // namespace sensor
