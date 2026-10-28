#include "sensor/BNO055.h"
#include <limits>
#include <iostream>
#include "common/Logger.h"
#include <cerrno>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <string_view>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <utility>

namespace
{
    std::string errnoMessage(const char *action)
    {
        std::ostringstream oss;
        oss << action << ": " << std::strerror(errno);
        return oss.str();
    }

    bool parseInt16(std::string_view text, std::int16_t &out)
    {
        if (text.empty())
        {
            return false;
        }

        // strtol cần chuỗi kết thúc bằng '\0'
        const std::string buffer(text);

        errno = 0;
        char *end = nullptr;

        const long value = std::strtol(buffer.c_str(), &end, 10);

        if (errno == ERANGE ||
            end == buffer.c_str() ||
            *end != '\0' ||
            value < std::numeric_limits<std::int16_t>::min() ||
            value > std::numeric_limits<std::int16_t>::max())
        {
            return false;
        }

        out = static_cast<std::int16_t>(value);
        return true;
    }
    std::string trim(std::string value)
    {
        const auto first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            return {};
        }

        const auto last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

} // namespace

bool BNO055::CalibrationProfile::operator==(
    const CalibrationProfile &rhs) const noexcept
{
    return accOffsetX == rhs.accOffsetX &&
           accOffsetY == rhs.accOffsetY &&
           accOffsetZ == rhs.accOffsetZ &&
           magOffsetX == rhs.magOffsetX &&
           magOffsetY == rhs.magOffsetY &&
           magOffsetZ == rhs.magOffsetZ &&
           gyrOffsetX == rhs.gyrOffsetX &&
           gyrOffsetY == rhs.gyrOffsetY &&
           gyrOffsetZ == rhs.gyrOffsetZ &&
           accRadius == rhs.accRadius &&
           magRadius == rhs.magRadius;
}

BNO055::BNO055(std::string i2cDevice, std::string calibrationFile)
    : i2cDevice_(std::move(i2cDevice)),
      calibrationFile_(std::move(calibrationFile)) {}

BNO055::~BNO055()
{
    close();
}

bool BNO055::initialize(std::chrono::milliseconds timeout)
{
    common::DeferredDeviceLog messages;
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    initialized_ = false;
    if (timeout.count() <= 0)
    {
        setError("Initialization timeout must be greater than zero");
        return false;
    }

    if (!openDevice())
    {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    if (!detectAddress(deadline))
    {
        return false;
    }

    if (!configure())
    {
        return false;
    }

    if (!measureYawRateDriftUnlocked(std::chrono::milliseconds(500)))
    {
        return false;
    }

    initialized_ = true;
    relativeYawDeg_ = 0.0;

    std::ifstream check(calibrationFile_);

    if (check.good())
    {
        check.close();

        if (!loadCalibrationFromFile())
        {
            clearError();
            common::Logger::Warn("BNO055","calibration","Tệp hiệu chuẩn không hợp lệ");
        }
    }
    return true;
}

void BNO055::close() noexcept
{
    if (fd_ >= 0)
    {
        ::close(fd_);
    }

    fd_ = -1;
    address_ = 0;
    initialized_ = false;
    mode_ = OperationMode::Config;
}

bool BNO055::isInitialized() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

std::uint8_t BNO055::address() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return address_;
}

std::string BNO055::lastError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

bool BNO055::openDevice()
{
    if (fd_ < 0) fd_ = ::open(i2cDevice_.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0)
    {
        setError(errnoMessage(("Cannot open " + i2cDevice_).c_str()));
        return false;
    }

    return true;
}

bool BNO055::detectAddress(
    std::chrono::steady_clock::time_point deadline)
{
    constexpr std::uint8_t addresses[] = {kAddressA, kAddressB};

    while (std::chrono::steady_clock::now() < deadline)
    {
        for (const auto candidate : addresses)
        {
            if (!selectAddress(candidate))
            {
                continue;
            }

            std::uint8_t chipId = 0;
            if (readRegister(REG_CHIP_ID, chipId) &&
                chipId == kChipIdExpected)
            {
                address_ = candidate;
                clearError();
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream oss;
    oss << "BNO055 not found on " << i2cDevice_
        << " at addresses 0x28 or 0x29 before timeout";
    setError(oss.str());
    return false;
}

bool BNO055::selectAddress(std::uint8_t address)
{
    if (::ioctl(fd_, I2C_SLAVE, address) < 0)
    {
        setError(errnoMessage("I2C_SLAVE ioctl failed"));
        return false;
    }

    address_ = address;
    return true;
}

bool BNO055::configure()
{
    if (!setMode(OperationMode::Config))
    {
        return false;
    }

    if (!writeRegister(REG_PAGE_ID, 0x00))
    {
        return false;
    }

    if (!writeRegister(REG_PWR_MODE, POWER_MODE_NORMAL))
    {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Units:
    // accel/linear/gravity = m/s^2
    // gyro = degrees/s
    // Euler = degrees
    // temperature = Celsius
    // Android orientation convention
    if (!writeRegister(REG_UNIT_SEL, 0x00))
    {
        return false;
    }

    if (!writeRegister(REG_AXIS_MAP_CONFIG, AXIS_MAP_DEFAULT) ||
        !writeRegister(REG_AXIS_MAP_SIGN, AXIS_SIGN_DEFAULT))
    {
        return false;
    }

    if (!setMode(OperationMode::Imu))
    {
        return false;
    }

    std::uint8_t sysStatus = 0;
    std::uint8_t sysError = 0;
    if (!readRegister(REG_SYS_STATUS, sysStatus) ||
        !readRegister(REG_SYS_ERR, sysError))
    {
        return false;
    }

    if (sysError != 0)
    {
        std::ostringstream oss;
        oss << "BNO055 system error after configuration: "
            << static_cast<unsigned>(sysError)
            << ", system status: "
            << static_cast<unsigned>(sysStatus);
        setError(oss.str());
        return false;
    }

    return true;
}

bool BNO055::setMode(OperationMode mode)
{
    if (!writeRegister(REG_OPR_MODE, static_cast<std::uint8_t>(mode)))
    {
        return false;
    }

    // Datasheet: operation -> CONFIG approximately 19 ms,
    // CONFIG -> operation approximately 7 ms.
    if (mode == OperationMode::Config)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    mode_ = mode;
    return true;
}

bool BNO055::writeRegister(std::uint8_t reg, std::uint8_t value)
{
    return writeRegisters(reg, &value, 1);
}

bool BNO055::readRegister(std::uint8_t reg, std::uint8_t &value)
{
    return readRegisters(reg, &value, 1);
}

bool BNO055::readRegisters(
    std::uint8_t startReg,
    std::uint8_t *data,
    std::size_t size)
{
    if (fd_ < 0 || address_ == 0)
    {
        setError("I2C device is not open or address is not selected");
        return false;
    }
    if (data == nullptr || size == 0 || size > 0xFFFF)
    {
        setError("Invalid I2C read buffer");
        return false;
    }

    std::uint8_t reg = startReg;

    i2c_msg messages[2]{};
    messages[0].addr = address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(size);
    messages[1].buf = data;

    i2c_rdwr_ioctl_data transaction{};
    transaction.msgs = messages;
    transaction.nmsgs = 2;

    if (::ioctl(fd_, I2C_RDWR, &transaction) < 0)
    {
        setError(errnoMessage("I2C register read failed"));
        return false;
    }

    return true;
}

bool BNO055::writeRegisters(
    std::uint8_t startReg,
    const std::uint8_t *data,
    std::size_t size)
{
    if (fd_ < 0 || address_ == 0)
    {
        setError("I2C device is not open or address is not selected");
        return false;
    }
    if (data == nullptr || size == 0 || size > 254)
    {
        setError("Invalid I2C write buffer");
        return false;
    }

    std::array<std::uint8_t, 255> buffer{};
    buffer[0] = startReg;
    std::memcpy(buffer.data() + 1, data, size);

    i2c_msg message{};
    message.addr = address_;
    message.flags = 0;
    message.len = static_cast<__u16>(size + 1);
    message.buf = buffer.data();

    i2c_rdwr_ioctl_data transaction{};
    transaction.msgs = &message;
    transaction.nmsgs = 1;

    if (::ioctl(fd_, I2C_RDWR, &transaction) < 0)
    {
        setError(errnoMessage("I2C register write failed"));
        return false;
    }

    return true;
}

bool BNO055::read(Data &data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    if (!readVector(REG_EUL_DATA, 16.0, data.eulerDeg) ||
        !readQuaternion(data.quaternion) ||
        !readVector(REG_ACC_DATA, 100.0, data.accelerationMps2) ||
        !readVector(REG_LIA_DATA, 100.0, data.linearAccelerationMps2) ||
        !readVector(REG_GRV_DATA, 100.0, data.gravityMps2) ||
        !readVector(REG_GYR_DATA, 16.0, data.gyroscopeDegPerSec) ||
        !readVector(REG_MAG_DATA, 16.0, data.magneticFieldMicroTesla))
    {
        return false;
    }

    std::uint8_t tempRaw = 0;
    if (!readRegister(REG_TEMP, tempRaw))
    {
        return false;
    }
    data.temperatureC =
        static_cast<double>(static_cast<std::int8_t>(tempRaw));

    std::uint8_t calibRaw = 0;
    if (!readRegister(REG_CALIB_STAT, calibRaw))
    {
        return false;
    }

    data.calibration.system = (calibRaw >> 6) & 0x03;
    data.calibration.gyroscope = (calibRaw >> 4) & 0x03;
    data.calibration.accelerometer = (calibRaw >> 2) & 0x03;
    data.calibration.magnetometer = calibRaw & 0x03;

    lastYawRateDegPerSec_ = data.gyroscopeDegPerSec.z;
    return true;
}

bool BNO055::measureYawRateDrift(std::chrono::milliseconds duration)
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    return measureYawRateDriftUnlocked(duration);
}

bool BNO055::measureYawRateDriftUnlocked(std::chrono::milliseconds duration)
{
    if (duration.count() <= 0)
    {
        setError("BNO055 yaw drift measurement duration must be positive");
        return false;
    }

    if (fd_ < 0 || address_ == 0)
    {
        setError("BNO055 I2C connection is not ready");
        return false;
    }

    std::uint64_t sampleCount = 0;
    double accumulatedYawRate = 0.0;
    const auto deadline = std::chrono::steady_clock::now() + duration;

    while (std::chrono::steady_clock::now() < deadline)
    {
        BNO055::Vector3 gyro;
        if (!readVector(REG_GYR_DATA, 16.0, gyro))
        {
            return false;
        }

        accumulatedYawRate += gyro.z;
        ++sampleCount;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (sampleCount == 0)
    {
        setError("BNO055 yaw drift measurement collected no samples");
        return false;
    }

    yawRateBiasDegPerSec_ = accumulatedYawRate / static_cast<double>(sampleCount);
    yawBiasMeasured_ = true;
    return true;
}

bool BNO055::integrateYawRate(double dtSeconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    if (!yawBiasMeasured_)
    {
        setError("BNO055 yaw bias has not been measured");
        return false;
    }

    if (dtSeconds <= 0.0)
    {
        setError("BNO055 integration interval must be positive");
        return false;
    }

    const double correctedYawRate = lastYawRateDegPerSec_ - yawRateBiasDegPerSec_;
    relativeYawDeg_ += correctedYawRate * dtSeconds;
    return true;
}

void BNO055::resetRelativeYaw(double yawDeg) noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    relativeYawDeg_ = yawDeg;
}

double BNO055::relativeYawDeg() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return relativeYawDeg_;
}

double BNO055::yawRateBiasDegPerSec() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return yawRateBiasDegPerSec_;
}

bool BNO055::readVector(
    std::uint8_t startReg,
    double scale,
    Vector3 &vector)
{
    std::array<std::uint8_t, 6> raw{};
    if (!readRegisters(startReg, raw.data(), raw.size()))
    {
        return false;
    }

    vector.x = static_cast<double>(decodeI16(raw.data() + 0)) / scale;
    vector.y = static_cast<double>(decodeI16(raw.data() + 2)) / scale;
    vector.z = static_cast<double>(decodeI16(raw.data() + 4)) / scale;
    return true;
}

bool BNO055::readQuaternion(Quaternion &quaternion)
{
    std::array<std::uint8_t, 8> raw{};
    if (!readRegisters(REG_QUA_DATA, raw.data(), raw.size()))
    {
        return false;
    }

    constexpr double scale = 16384.0;
    quaternion.w = static_cast<double>(decodeI16(raw.data() + 0)) / scale;
    quaternion.x = static_cast<double>(decodeI16(raw.data() + 2)) / scale;
    quaternion.y = static_cast<double>(decodeI16(raw.data() + 4)) / scale;
    quaternion.z = static_cast<double>(decodeI16(raw.data() + 6)) / scale;
    return true;
}

bool BNO055::readCalibrationStatus(CalibrationStatus &status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    std::uint8_t raw = 0;
    if (!readRegister(REG_CALIB_STAT, raw))
    {
        return false;
    }

    status.system = (raw >> 6) & 0x03;
    status.gyroscope = (raw >> 4) & 0x03;
    status.accelerometer = (raw >> 2) & 0x03;
    status.magnetometer = raw & 0x03;
    return true;
}

bool BNO055::saveCalibrationIfFullyCalibrated()
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    std::uint8_t rawStatus = 0;
    if (!readRegister(REG_CALIB_STAT, rawStatus))
    {
        return false;
    }

    CalibrationStatus status{
        static_cast<std::uint8_t>((rawStatus >> 6) & 0x03),
        static_cast<std::uint8_t>((rawStatus >> 4) & 0x03),
        static_cast<std::uint8_t>((rawStatus >> 2) & 0x03),
        static_cast<std::uint8_t>(rawStatus & 0x03)};

    if (!status.fullyCalibrated())
    {
        return true; // Not an error; calibration is still progressing.
    }

    CalibrationProfile profile;
    if (!readCalibrationProfile(profile))
    {
        return false;
    }

    if (haveLastSavedProfile_ && profile == lastSavedProfile_)
    {
        return true;
    }

    // Inline save avoids re-locking mutex_.
    std::ofstream file(calibrationFile_, std::ios::trunc);
    if (!file.is_open())
    {
        setError("Cannot open calibration file for writing: " +
                 calibrationFile_);
        return false;
    }

    file << "BNO055_CALIB_V1\n";
    file << "ADDRESS=" << static_cast<unsigned>(address_) << "\n";
    file << "ACC_OFFSET_X=" << profile.accOffsetX << "\n";
    file << "ACC_OFFSET_Y=" << profile.accOffsetY << "\n";
    file << "ACC_OFFSET_Z=" << profile.accOffsetZ << "\n";
    file << "MAG_OFFSET_X=" << profile.magOffsetX << "\n";
    file << "MAG_OFFSET_Y=" << profile.magOffsetY << "\n";
    file << "MAG_OFFSET_Z=" << profile.magOffsetZ << "\n";
    file << "GYR_OFFSET_X=" << profile.gyrOffsetX << "\n";
    file << "GYR_OFFSET_Y=" << profile.gyrOffsetY << "\n";
    file << "GYR_OFFSET_Z=" << profile.gyrOffsetZ << "\n";
    file << "ACC_RADIUS=" << profile.accRadius << "\n";
    file << "MAG_RADIUS=" << profile.magRadius << "\n";

    if (!file.good())
    {
        setError("Failed while writing calibration file: " +
                 calibrationFile_);
        return false;
    }

    lastSavedProfile_ = profile;
    haveLastSavedProfile_ = true;
    return true;
}

bool BNO055::loadCalibrationFromFile()
{
    // initialize() already holds mutex_; this public method may also be called
    // directly, so locking is intentionally omitted here to avoid deadlock.
    // Use this method only when no other thread is accessing the instance.
    clearError();

    if (fd_ < 0 || address_ == 0)
    {
        setError("BNO055 I2C connection is not ready");
        return false;
    }

    std::ifstream file(calibrationFile_);
    if (!file.is_open())
    {
        setError("Cannot open calibration file: " + calibrationFile_);
        return false;
    }

    std::string header;
    if (!std::getline(file, header) ||
        trim(header) != "BNO055_CALIB_V1")
    {
        setError("Invalid calibration file header");
        return false;
    }

    CalibrationProfile profile;
    bool seen[11]{};

    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line.front() == '#')
        {
            continue;
        }

        const auto pos = line.find('=');
        if (pos == std::string::npos)
        {
            setError("Invalid calibration line: " + line);
            return false;
        }

        const std::string key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));

        if (key == "ADDRESS")
        {
            continue; // Informational only; auto-detection remains authoritative.
        }

        std::int16_t parsed = 0;
        if (!parseInt16(value, parsed))
        {
            setError("Invalid decimal calibration value for " + key);
            return false;
        }

        if (key == "ACC_OFFSET_X")
        {
            profile.accOffsetX = parsed;
            seen[0] = true;
        }
        else if (key == "ACC_OFFSET_Y")
        {
            profile.accOffsetY = parsed;
            seen[1] = true;
        }
        else if (key == "ACC_OFFSET_Z")
        {
            profile.accOffsetZ = parsed;
            seen[2] = true;
        }
        else if (key == "MAG_OFFSET_X")
        {
            profile.magOffsetX = parsed;
            seen[3] = true;
        }
        else if (key == "MAG_OFFSET_Y")
        {
            profile.magOffsetY = parsed;
            seen[4] = true;
        }
        else if (key == "MAG_OFFSET_Z")
        {
            profile.magOffsetZ = parsed;
            seen[5] = true;
        }
        else if (key == "GYR_OFFSET_X")
        {
            profile.gyrOffsetX = parsed;
            seen[6] = true;
        }
        else if (key == "GYR_OFFSET_Y")
        {
            profile.gyrOffsetY = parsed;
            seen[7] = true;
        }
        else if (key == "GYR_OFFSET_Z")
        {
            profile.gyrOffsetZ = parsed;
            seen[8] = true;
        }
        else if (key == "ACC_RADIUS")
        {
            profile.accRadius = parsed;
            seen[9] = true;
        }
        else if (key == "MAG_RADIUS")
        {
            profile.magRadius = parsed;
            seen[10] = true;
        }
        else
        {
            setError("Unknown calibration field: " + key);
            return false;
        }
    }

    for (bool fieldSeen : seen)
    {
        if (!fieldSeen)
        {
            setError("Calibration file is missing one or more fields");
            return false;
        }
    }

    if (!writeCalibrationProfile(profile))
    {
        return false;
    }

    lastSavedProfile_ = profile;
    haveLastSavedProfile_ = true;
    return true;
}

bool BNO055::saveCalibrationToFile()
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    CalibrationProfile profile;
    if (!readCalibrationProfile(profile))
    {
        return false;
    }

    std::ofstream file(calibrationFile_, std::ios::trunc);
    if (!file.is_open())
    {
        setError("Cannot open calibration file for writing: " +
                 calibrationFile_);
        return false;
    }

    file << "BNO055_CALIB_V1\n";
    file << "ADDRESS=" << static_cast<unsigned>(address_) << "\n";
    file << "ACC_OFFSET_X=" << profile.accOffsetX << "\n";
    file << "ACC_OFFSET_Y=" << profile.accOffsetY << "\n";
    file << "ACC_OFFSET_Z=" << profile.accOffsetZ << "\n";
    file << "MAG_OFFSET_X=" << profile.magOffsetX << "\n";
    file << "MAG_OFFSET_Y=" << profile.magOffsetY << "\n";
    file << "MAG_OFFSET_Z=" << profile.magOffsetZ << "\n";
    file << "GYR_OFFSET_X=" << profile.gyrOffsetX << "\n";
    file << "GYR_OFFSET_Y=" << profile.gyrOffsetY << "\n";
    file << "GYR_OFFSET_Z=" << profile.gyrOffsetZ << "\n";
    file << "ACC_RADIUS=" << profile.accRadius << "\n";
    file << "MAG_RADIUS=" << profile.magRadius << "\n";

    if (!file.good())
    {
        setError("Failed while writing calibration file: " +
                 calibrationFile_);
        return false;
    }

    lastSavedProfile_ = profile;
    haveLastSavedProfile_ = true;
    return true;
}

bool BNO055::readCalibrationProfile(CalibrationProfile &profile)
{
    OperationMode previousMode;
    if (!switchToConfig(previousMode))
    {
        return false;
    }

    std::array<std::uint8_t, 22> raw{};
    const bool readOk =
        readRegisters(REG_CALIB_START, raw.data(), raw.size());
    const bool restoreOk = restoreMode(previousMode);

    if (!readOk || !restoreOk)
    {
        return false;
    }

    profile = decodeProfile(raw);
    return true;
}

bool BNO055::writeCalibrationProfile(
    const CalibrationProfile &profile)
{
    OperationMode previousMode;
    if (!switchToConfig(previousMode))
    {
        return false;
    }

    const auto raw = encodeProfile(profile);
    const bool writeOk =
        writeRegisters(REG_CALIB_START, raw.data(), raw.size());
    const bool restoreOk = restoreMode(previousMode);

    return writeOk && restoreOk;
}

bool BNO055::setAxisRemap(
    std::uint8_t axisMapConfig,
    std::uint8_t axisMapSign)
{
    std::lock_guard<std::mutex> lock(mutex_);
    clearError();

    if (!initialized_)
    {
        setError("BNO055 is not initialized");
        return false;
    }

    OperationMode previousMode;
    if (!switchToConfig(previousMode))
    {
        return false;
    }

    const bool writeOk =
        writeRegister(REG_AXIS_MAP_CONFIG, axisMapConfig) &&
        writeRegister(REG_AXIS_MAP_SIGN, axisMapSign);
    const bool restoreOk = restoreMode(previousMode);

    return writeOk && restoreOk;
}

std::int16_t BNO055::decodeI16(
    const std::uint8_t *data) noexcept
{
    const std::uint16_t value =
        static_cast<std::uint16_t>(data[0]) |
        (static_cast<std::uint16_t>(data[1]) << 8);
    return static_cast<std::int16_t>(value);
}

void BNO055::encodeI16(
    std::int16_t value,
    std::uint8_t *data) noexcept
{
    const auto raw = static_cast<std::uint16_t>(value);
    data[0] = static_cast<std::uint8_t>(raw & 0xFF);
    data[1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
}

BNO055::CalibrationProfile BNO055::decodeProfile(
    const std::array<std::uint8_t, 22> &raw) noexcept
{
    CalibrationProfile profile;
    profile.accOffsetX = decodeI16(raw.data() + 0);
    profile.accOffsetY = decodeI16(raw.data() + 2);
    profile.accOffsetZ = decodeI16(raw.data() + 4);
    profile.magOffsetX = decodeI16(raw.data() + 6);
    profile.magOffsetY = decodeI16(raw.data() + 8);
    profile.magOffsetZ = decodeI16(raw.data() + 10);
    profile.gyrOffsetX = decodeI16(raw.data() + 12);
    profile.gyrOffsetY = decodeI16(raw.data() + 14);
    profile.gyrOffsetZ = decodeI16(raw.data() + 16);
    profile.accRadius = decodeI16(raw.data() + 18);
    profile.magRadius = decodeI16(raw.data() + 20);
    return profile;
}

std::array<std::uint8_t, 22> BNO055::encodeProfile(
    const CalibrationProfile &profile) noexcept
{
    std::array<std::uint8_t, 22> raw{};
    encodeI16(profile.accOffsetX, raw.data() + 0);
    encodeI16(profile.accOffsetY, raw.data() + 2);
    encodeI16(profile.accOffsetZ, raw.data() + 4);
    encodeI16(profile.magOffsetX, raw.data() + 6);
    encodeI16(profile.magOffsetY, raw.data() + 8);
    encodeI16(profile.magOffsetZ, raw.data() + 10);
    encodeI16(profile.gyrOffsetX, raw.data() + 12);
    encodeI16(profile.gyrOffsetY, raw.data() + 14);
    encodeI16(profile.gyrOffsetZ, raw.data() + 16);
    encodeI16(profile.accRadius, raw.data() + 18);
    encodeI16(profile.magRadius, raw.data() + 20);
    return raw;
}

bool BNO055::switchToConfig(OperationMode &previousMode)
{
    previousMode = mode_;
    if (mode_ == OperationMode::Config)
    {
        return true;
    }
    return setMode(OperationMode::Config);
}

bool BNO055::restoreMode(OperationMode previousMode)
{
    if (previousMode == mode_)
    {
        return true;
    }
    return setMode(previousMode);
}

void BNO055::setError(std::string message)
{
    lastError_ = std::move(message);
}

void BNO055::clearError()
{
    lastError_.clear();
}
