#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

class BNO055
{
public:
    struct Vector3
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
    };

    struct Quaternion
    {
        double w{1.0};
        double x{0.0};
        double y{0.0};
        double z{0.0};
    };

    struct CalibrationStatus
    {
        std::uint8_t system{0};
        std::uint8_t gyroscope{0};
        std::uint8_t accelerometer{0};
        std::uint8_t magnetometer{0};

        [[nodiscard]] bool fullyCalibrated() const noexcept
        {
            return system == 3 && gyroscope == 3 &&
                   accelerometer == 3 && magnetometer == 3;
        }
    };

    struct Data
    {
        Vector3 eulerDeg;
        Quaternion quaternion;

        Vector3 accelerationMps2;
        Vector3 linearAccelerationMps2;
        Vector3 gravityMps2;

        Vector3 gyroscopeDegPerSec;
        Vector3 magneticFieldMicroTesla;

        double temperatureC{0.0};
        CalibrationStatus calibration;
    };

    struct CalibrationProfile
    {
        std::int16_t accOffsetX{0};
        std::int16_t accOffsetY{0};
        std::int16_t accOffsetZ{0};

        std::int16_t magOffsetX{0};
        std::int16_t magOffsetY{0};
        std::int16_t magOffsetZ{0};

        std::int16_t gyrOffsetX{0};
        std::int16_t gyrOffsetY{0};
        std::int16_t gyrOffsetZ{0};

        std::int16_t accRadius{0};
        std::int16_t magRadius{0};

        bool operator==(const CalibrationProfile &rhs) const noexcept;
        bool operator!=(const CalibrationProfile &rhs) const noexcept
        {
            return !(*this == rhs);
        }
    };

    explicit BNO055(
        std::string i2cDevice = "/dev/i2c-1",
        std::string calibrationFile = "/home/car4/Desktop/CAR3/include/sensor/bno055_calibration.txt");

    ~BNO055();

    BNO055(const BNO055 &) = delete;
    BNO055 &operator=(const BNO055 &) = delete;

    bool initialize(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    void close() noexcept;

    [[nodiscard]] bool isInitialized() const noexcept;
    [[nodiscard]] std::uint8_t address() const noexcept;
    [[nodiscard]] std::string lastError() const;

    bool read(Data &data);

    bool readCalibrationStatus(CalibrationStatus &status);

    bool saveCalibrationIfFullyCalibrated();

    bool loadCalibrationFromFile();
    bool saveCalibrationToFile();

    bool readCalibrationProfile(CalibrationProfile &profile);
    bool writeCalibrationProfile(const CalibrationProfile &profile);

    bool measureYawRateDrift(
        std::chrono::milliseconds duration = std::chrono::milliseconds(500));

    bool integrateYawRate(double dtSeconds);
    void resetRelativeYaw(double yawDeg = 0.0) noexcept;
    [[nodiscard]] double relativeYawDeg() const noexcept;
    [[nodiscard]] double yawRateBiasDegPerSec() const noexcept;

    bool setAxisRemap(
        std::uint8_t axisMapConfig,
        std::uint8_t axisMapSign);

private:
    enum class OperationMode : std::uint8_t
    {
        Config = 0x00,
        Ndof = 0x0C,
        Imu = 0x08
    };

    static constexpr std::uint8_t kAddressA = 0x28;
    static constexpr std::uint8_t kAddressB = 0x29;
    static constexpr std::uint8_t kChipIdExpected = 0xA0;

    static constexpr std::uint8_t REG_CHIP_ID = 0x00;
    static constexpr std::uint8_t REG_PAGE_ID = 0x07;
    static constexpr std::uint8_t REG_ACC_DATA = 0x08;
    static constexpr std::uint8_t REG_MAG_DATA = 0x0E;
    static constexpr std::uint8_t REG_GYR_DATA = 0x14;
    static constexpr std::uint8_t REG_EUL_DATA = 0x1A;
    static constexpr std::uint8_t REG_QUA_DATA = 0x20;
    static constexpr std::uint8_t REG_LIA_DATA = 0x28;
    static constexpr std::uint8_t REG_GRV_DATA = 0x2E;
    static constexpr std::uint8_t REG_TEMP = 0x34;
    static constexpr std::uint8_t REG_CALIB_STAT = 0x35;
    static constexpr std::uint8_t REG_SYS_STATUS = 0x39;
    static constexpr std::uint8_t REG_SYS_ERR = 0x3A;
    static constexpr std::uint8_t REG_UNIT_SEL = 0x3B;
    static constexpr std::uint8_t REG_OPR_MODE = 0x3D;
    static constexpr std::uint8_t REG_PWR_MODE = 0x3E;
    static constexpr std::uint8_t REG_SYS_TRIGGER = 0x3F;
    static constexpr std::uint8_t REG_AXIS_MAP_CONFIG = 0x41;
    static constexpr std::uint8_t REG_AXIS_MAP_SIGN = 0x42;
    static constexpr std::uint8_t REG_CALIB_START = 0x55;

    static constexpr std::uint8_t POWER_MODE_NORMAL = 0x00;
    static constexpr std::uint8_t AXIS_MAP_DEFAULT = 0x24;
    static constexpr std::uint8_t AXIS_SIGN_DEFAULT = 0x00;

    bool openDevice();
    bool detectAddress(std::chrono::steady_clock::time_point deadline);
    bool selectAddress(std::uint8_t address);
    bool configure();

    bool setMode(OperationMode mode);
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegister(std::uint8_t reg, std::uint8_t &value);
    bool readRegisters(std::uint8_t startReg, std::uint8_t *data, std::size_t size);
    bool writeRegisters(std::uint8_t startReg, const std::uint8_t *data, std::size_t size);

    bool readVector(std::uint8_t startReg, double scale, Vector3 &vector);
    bool readQuaternion(Quaternion &quaternion);
    bool measureYawRateDriftUnlocked(std::chrono::milliseconds duration);

    static std::int16_t decodeI16(const std::uint8_t *data) noexcept;
    static void encodeI16(std::int16_t value, std::uint8_t *data) noexcept;

    static CalibrationProfile decodeProfile(
        const std::array<std::uint8_t, 22> &raw) noexcept;
    static std::array<std::uint8_t, 22> encodeProfile(
        const CalibrationProfile &profile) noexcept;

    bool switchToConfig(OperationMode &previousMode);
    bool restoreMode(OperationMode previousMode);

    void setError(std::string message);
    void clearError();

    std::string i2cDevice_;
    std::string calibrationFile_;

    int fd_{-1};
    std::uint8_t address_{0};
    OperationMode mode_{OperationMode::Config};
    bool initialized_{false};

    CalibrationProfile lastSavedProfile_{};
    bool haveLastSavedProfile_{false};

    double yawRateBiasDegPerSec_{0.0};
    double lastYawRateDegPerSec_{0.0};
    double relativeYawDeg_{0.0};
    bool yawBiasMeasured_{false};

    mutable std::mutex mutex_;
    std::string lastError_;
};
