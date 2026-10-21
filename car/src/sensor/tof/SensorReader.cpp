#include "SensorReader.h"
#include "StTransport.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <chrono>

extern "C" {
#include "vl53l0x_api.h"
#undef _LOG_FUNCTION_START
#undef _LOG_FUNCTION_END
#undef _LOG_FUNCTION_END_FMT
#undef PALDevDataSet
#undef PALDevDataGet
#include "vl53l1_api.h"
#include "vl53l1_ll_device.h"
}

namespace tof {
struct SensorReader::Impl {
    Hardware* hardware = nullptr;
    std::array<SensorType, 4> types;
    std::array<VL53L0X_Dev_t, 4> l0{};
    std::array<VL53L1_Dev_t, 4> l1{};
    std::array<bool, 4> started{};
};
namespace {
bool isWarning(int status) {
    return status == -28 || status == -29 || status == -30;
}
int readStatus(int status, std::size_t slot, const char* operation) {
    if (!isWarning(status)) return status;
    std::cerr << "[WARN] BOARD " << kSlots[slot].boardPin << ' ' << operation << " ST=" << status << '\n';
    return 0;
}
void check(int status, std::size_t slot, const char* operation) {
    if (status == 0 || isWarning(status)) return;
    std::ostringstream error;
    error << "BOARD " << kSlots[slot].boardPin << ": " << operation << " failed, ST error=" << status;
    if (!stLastError().empty()) error << "; transport: " << stLastError();
    throw std::runtime_error(error.str());
}
template<class Function>
int runStep(std::ostream& log, std::size_t slot, const char* operation, Function function) {
    const auto start = std::chrono::steady_clock::now();
    log << "[INIT] BOARD " << kSlots[slot].boardPin << ' ' << operation << " BEGIN" << std::endl;
    stSetDeadlineMs(kInitStepTimeoutMs);
    int status = 0;
    try { status = function(); }
    catch (...) { stClearDeadline(); throw; }
    const bool timedOut = stDeadlineExpired();
    stClearDeadline();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    log << "[INIT] BOARD " << kSlots[slot].boardPin << ' ' << operation
        << (timedOut ? " TIMEOUT" : (status == 0 ? " OK" : isWarning(status) ? " WARNING" : " FAILED"))
        << " ST=" << status << " elapsed=" << elapsed << " ms" << std::endl;
    if (timedOut) {
        std::ostringstream error;
        error << "BOARD " << kSlots[slot].boardPin << ": " << operation
              << " TIMEOUT (limit " << kInitStepTimeoutMs << " ms)";
        if (!stLastError().empty()) error << "; transport: " << stLastError();
        throw std::runtime_error(error.str());
    }
    check(status, slot, operation);
    return status;
}
void diagnoseL1(Hardware& hardware, std::size_t slot, std::ostream& log) {
    const uint8_t address = kSlots[slot].address;
    const auto word = [&](uint16_t reg) {
        uint8_t bytes[2]{};
        hardware.readBlock(address, reg, true, bytes, 2);
        const uint16_t burst = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
        const uint16_t separate = static_cast<uint16_t>(
            (hardware.readByte(address, reg, true) << 8) |
            hardware.readByte(address, static_cast<uint16_t>(reg + 1), true));
        if (burst != separate) {
            std::ostringstream message;
            message << "BOARD " << kSlots[slot].boardPin << " inconsistent I2C word at register 0x"
                    << std::hex << reg << ": block=0x" << burst << " bytes=0x" << separate;
            throw std::runtime_error(message.str());
        }
        return burst;
    };
    const auto id = word(0x010F);
    const auto revision = hardware.readByte(address, 0x0111, true);
    const auto fastOsc = word(0x0006);
    const auto oscCal = word(0x00DE);
    const auto boot = hardware.readByte(address, 0x00E5, true);
    log << "[DIAG] BOARD " << kSlots[slot].boardPin << " I2C=0x" << std::hex
        << static_cast<unsigned>(address) << " model/module=0x" << id
        << " revision=0x" << static_cast<unsigned>(revision)
        << " fast_osc=0x" << fastOsc << " osc_cal=0x" << oscCal
        << " boot=0x" << static_cast<unsigned>(boot) << std::dec << '\n';
    if (fastOsc == 0 || oscCal == 0)
        log << "[DIAG] BOARD " << kSlots[slot].boardPin
            << " oscillator register is zero; ST initialization requires valid calibration.\n";
}
}
SensorReader::SensorReader(Hardware& hardware, const std::array<SensorType, 4>& types)
    : impl_(new Impl) {
    impl_->types = types;
    impl_->hardware = &hardware;
    setStHardware(&hardware);
    for (std::size_t i = 0; i < types.size(); ++i) {
        impl_->l0[i].I2cDevAddr = static_cast<uint8_t>(kSlots[i].address << 1);
        impl_->l1[i].i2c_slave_address = static_cast<uint8_t>(kSlots[i].address << 1);
        impl_->l0[i].comms_type = 1; // ST I2C
        impl_->l1[i].comms_type = 1;
        impl_->l0[i].comms_speed_khz = 100;
        impl_->l1[i].comms_speed_khz = 100;
    }
}
SensorReader::~SensorReader() {
    stop(std::cerr);
    setStHardware(nullptr);
}
void SensorReader::start(std::ostream& log) {
    std::array<bool, 4> configured{};
    for (std::size_t i = 0; i < kSlots.size(); ++i) {
        if (impl_->started[i]) throw std::runtime_error("Ranging already started");
        if (impl_->types[i] == SensorType::Unknown) continue;
        try {
            if (impl_->types[i] == SensorType::L0) {
                auto dev = &impl_->l0[i];
                runStep(log, i, "DataInit", [&] { return VL53L0X_DataInit(dev); });
                runStep(log, i, "StaticInit", [&] { return VL53L0X_StaticInit(dev); });
                uint32_t spadCount = 0; uint8_t aperture = 0;
                runStep(log, i, "RefSpadManagement", [&] { return VL53L0X_PerformRefSpadManagement(dev, &spadCount, &aperture); });
                uint8_t vhv = 0, phase = 0;
                runStep(log, i, "RefCalibration", [&] { return VL53L0X_PerformRefCalibration(dev, &vhv, &phase); });
                runStep(log, i, "SetDeviceMode", [&] { return VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); });
                runStep(log, i, "TimingBudget", [&] { return VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 50000); });
                runStep(log, i, "ClearInterrupt", [&] { return VL53L0X_ClearInterruptMask(dev, 0); });
            } else if (impl_->types[i] == SensorType::L1) {
                auto dev = &impl_->l1[i];
                dev->Data.LLData.wait_method = VL53L1_WAIT_METHOD_BLOCKING;
                runStep(log, i, "WaitDeviceBooted", [&] { return VL53L1_WaitDeviceBooted(dev); });
                diagnoseL1(*impl_->hardware, i, log);
                const int initStatus = runStep(log, i, "DataInit", [&] { return VL53L1_DataInit(dev); });
                log << "[DIAG] BOARD " << kSlots[i].boardPin
                    << " DataInit=" << initStatus << " cached_fast_osc=0x" << std::hex
                    << dev->Data.LLData.stat_nvm.osc_measured__fast_osc__frequency
                    << " cached_osc_cal=0x" << dev->Data.LLData.dbg_results.result__osc_calibrate_val
                    << std::dec << '\n';
                check(initStatus, i, "DataInit");
                runStep(log, i, "StaticInit", [&] { return VL53L1_StaticInit(dev); });
                runStep(log, i, "RefSpadManagement", [&] { return VL53L1_PerformRefSpadManagement(dev); });
                runStep(log, i, "SetDistanceMode", [&] { return VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT); });
                runStep(log, i, "TimingBudget", [&] { return VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 50000); });
                runStep(log, i, "InterMeasurementPeriod", [&] { return VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 100); });
            } else {
                throw std::runtime_error("Cannot start an unidentified sensor");
            }
            configured[i] = true;
        } catch (const std::exception& error) {
            log << "[ERROR] BOARD " << kSlots[i].boardPin << " init: " << error.what() << '\n';
        }
    }
    // Begin ranging after all devices have been configured.
    for (std::size_t i = 0; i < kSlots.size(); ++i) {
        if (!configured[i]) continue;
        try {
            // Include this device in cleanup even if a partially completed start fails.
            impl_->started[i] = true;
            runStep(log, i, "StartMeasurement", [&] {
                return impl_->types[i] == SensorType::L0
                    ? VL53L0X_StartMeasurement(&impl_->l0[i]) : VL53L1_StartMeasurement(&impl_->l1[i]);
            });
            log << "[RANGE] BOARD " << kSlots[i].boardPin << ' ' << typeName(impl_->types[i]) << " started\n";
        } catch (const std::exception& error) {
            log << "[ERROR] BOARD " << kSlots[i].boardPin << " start: " << error.what() << '\n';
            try {
                if (impl_->types[i] == SensorType::L0) VL53L0X_StopMeasurement(&impl_->l0[i]);
                else VL53L1_StopMeasurement(&impl_->l1[i]);
            } catch (...) {}
            impl_->started[i] = false;
        }
    }
    for (const bool started : impl_->started) if (started) return;
    throw std::runtime_error("No TOF sensor initialized successfully");
}
bool SensorReader::initialized(std::size_t slot) const {
    return impl_->started.at(slot);
}
Measurement SensorReader::read(std::size_t slot) {
    Measurement result;
    if (!impl_->started.at(slot)) throw std::runtime_error("Ranging has not started");
    uint8_t ready = 0;
    if (impl_->types[slot] == SensorType::L0) {
        auto dev = &impl_->l0[slot];
        result.apiError = readStatus(VL53L0X_GetMeasurementDataReady(dev, &ready), slot, "read");
        if (result.apiError != 0 || !ready) return result;
        VL53L0X_RangingMeasurementData_t data{};
        result.apiError = readStatus(VL53L0X_GetRangingMeasurementData(dev, &data), slot, "read");
        if (result.apiError != 0) return result;
        result.rangeStatus = data.RangeStatus;
        result.distanceMm = data.RangeMilliMeter;
        result.apiError = readStatus(VL53L0X_ClearInterruptMask(dev, 0), slot, "read");
    } else {
        auto dev = &impl_->l1[slot];
        result.apiError = readStatus(VL53L1_GetMeasurementDataReady(dev, &ready), slot, "read");
        if (result.apiError != 0 || !ready) return result;
        VL53L1_RangingMeasurementData_t data{};
        result.apiError = readStatus(VL53L1_GetRangingMeasurementData(dev, &data), slot, "read");
        if (result.apiError != 0) return result;
        result.rangeStatus = data.RangeStatus;
        result.distanceMm = data.RangeMilliMeter;
        result.apiError = readStatus(VL53L1_ClearInterruptAndStartMeasurement(dev), slot, "read");
    }
    result.ready = result.apiError == 0;
    return result;
}
bool SensorReader::stop(std::ostream& log) {
    bool ok = true;
    for (std::size_t i = 0; i < kSlots.size(); ++i) {
        if (!impl_->started[i]) continue;
        const int status = impl_->types[i] == SensorType::L0
            ? VL53L0X_StopMeasurement(&impl_->l0[i]) : VL53L1_StopMeasurement(&impl_->l1[i]);
        if (status != 0) {
            log << (isWarning(status) ? "[WARN] BOARD " : "[ERROR] BOARD ") << kSlots[i].boardPin << " StopMeasurement ST=" << status << '\n';
            if (!isWarning(status)) ok = false;
        }
        impl_->started[i] = false;
    }
    return ok;
}
}
