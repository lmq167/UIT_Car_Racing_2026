#include "sensor/BNO055.h"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

enum class RunMode
{
    Euler,
    Info,
    All,
    Calibration
};

void printCalibrationItem(const char *name, std::uint8_t value)
{
    std::cout << "  " << name << ": "
              << static_cast<unsigned>(value)
              << "/3";

    if (value == 3)
    {
        std::cout << " [DAT]";
    }
    else
    {
        std::cout << " [CHUA DAT]";
    }

    std::cout << '\n';
}

void printEuler(const BNO055::Data &data)
{
    std::cout << std::fixed
              << std::setprecision(2)
              << "Yaw=" << data.eulerDeg.x
              << " Roll=" << data.eulerDeg.y
              << " Pitch=" << data.eulerDeg.z
              << '\n';
}

void printCalibrationStatus(const BNO055::CalibrationStatus &calib)
{
    std::cout << "Calibration status:\n";

    printCalibrationItem("SYSTEM", calib.system);
    printCalibrationItem("GYROSCOPE", calib.gyroscope);
    printCalibrationItem("ACCELEROMETER", calib.accelerometer);
    printCalibrationItem("MAGNETOMETER", calib.magnetometer);
}

void printInfo(const BNO055 &bno, const BNO055::Data &data)
{
    std::cout << std::fixed
              << std::setprecision(2)
              << "I2C address: 0x"
              << std::hex
              << static_cast<unsigned>(bno.address())
              << std::dec
              << '\n';

    std::cout << "Temperature: "
              << data.temperatureC
              << " C\n";

    printCalibrationStatus(data.calibration);
}

void printAll(const BNO055::Data &data)
{
    std::cout << std::fixed << std::setprecision(2);

    const double vehicleRoll  = data.eulerDeg.z;
    const double vehiclePitch = -data.eulerDeg.y;

    double vehicleYaw = 360.0 - data.eulerDeg.x;

    if (vehicleYaw >= 360.0)
    {
        vehicleYaw -= 360.0;
    }

    std::cout << "Euler:\n"
           << "  Yaw   = " << vehicleYaw   << " deg\n"
           << "  Roll  = " << vehicleRoll  << " deg\n"
           << "  Pitch = " << vehiclePitch << " deg\n";

    std::cout << "Gyroscope:\n"
              << "  X = " << data.gyroscopeDegPerSec.x << " deg/s\n"
              << "  Y = " << data.gyroscopeDegPerSec.y << " deg/s\n"
              << "  Z = " << data.gyroscopeDegPerSec.z << " deg/s\n";

    std::cout << "Acceleration:\n"
              << "  X = " << data.accelerationMps2.x << " m/s^2\n"
              << "  Y = " << data.accelerationMps2.y << " m/s^2\n"
              << "  Z = " << data.accelerationMps2.z << " m/s^2\n";
}

void runCalibrationMode(BNO055 &bno)
{
    std::cout << "\nBNO055 calibration mode\n\n";

    std::cout << "Calibration instructions:\n"
              << "- GYR: Keep the sensor completely still.\n"
              << "- ACC: Place the sensor steadily on all 6 sides.\n"
              << "- MAG: Move the sensor in a figure-eight pattern.\n\n";

    constexpr int requiredStableSamples = 20;

    int fullyCalibratedSamples = 0;
    bool calibrationSaved = false;

    while (true)
    {
        BNO055::Data data;

        if (!bno.read(data))
        {
            std::cerr << "BNO055 read failed: "
                      << bno.lastError()
                      << '\n';

            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));

            continue;
        }

        const auto &calib = data.calibration;

        printEuler(data);
        printCalibrationStatus(calib);

        if (!calibrationSaved)
        {
            if (calib.fullyCalibrated())
            {
                ++fullyCalibratedSamples;

                std::cout << "Stable samples: "
                          << fullyCalibratedSamples
                          << '/'
                          << requiredStableSamples
                          << '\n';

                if (fullyCalibratedSamples >= requiredStableSamples)
                {
                    if (bno.saveCalibrationIfFullyCalibrated())
                    {
                        calibrationSaved = true;

                        std::cout
                            << "\nCALIBRATION COMPLETED\n"
                            << "Calibration data was saved successfully.\n";
                    }
                    else
                    {
                        std::cerr
                            << "Save calibration failed: "
                            << bno.lastError()
                            << '\n';

                        fullyCalibratedSamples = 0;
                    }
                }
            }
            else
            {
                fullyCalibratedSamples = 0;

                std::cout << "Continue calibration:\n";

                if (calib.gyroscope < 3)
                {
                    std::cout
                        << "  - Keep the sensor still for GYR.\n";
                }

                if (calib.accelerometer < 3)
                {
                    std::cout
                        << "  - Place the sensor on all 6 sides for ACC.\n";
                }

                if (calib.magnetometer < 3)
                {
                    std::cout
                        << "  - Move the sensor in a figure-eight pattern "
                           "for MAG.\n";
                }

                if (calib.system < 3)
                {
                    std::cout
                        << "  - SYSTEM is not fully calibrated yet.\n";
                }
            }
        }
        else
        {
            std::cout << "Calibration has already been saved.\n";
        }

        std::cout << '\n';

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }
}

void runDisplayMode(BNO055 &bno, RunMode mode)
{
    auto previousTime = std::chrono::steady_clock::now();

    while (true)
    {
        BNO055::Data data;

        if (!bno.read(data))
        {
            std::cerr << "BNO055 read failed: "
                      << bno.lastError()
                      << '\n';

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            previousTime = std::chrono::steady_clock::now();
            continue;
        }

        const auto currentTime = std::chrono::steady_clock::now();

        const double dtSeconds =
            std::chrono::duration<double>(
                currentTime - previousTime).count();

        previousTime = currentTime;

        if (!bno.integrateYawRate(dtSeconds))
        {
            std::cerr << "BNO055 relative yaw update failed: "
                      << bno.lastError()
                      << '\n';

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            continue;
        }

        switch (mode)
        {
        case RunMode::Euler:
            printEuler(data);
            break;

        case RunMode::Info:
            printInfo(bno, data);
            break;

        case RunMode::All:
            printAll(data);
            break;

        case RunMode::Calibration:
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void printUsage(const char *programName)
{
    std::cerr
        << "Usage:\n"
        << "  " << programName
        << "          Display Euler angles only\n"

        << "  " << programName
        << " info     Display sensor information\n"

        << "  " << programName
        << " all      Display gyro, acceleration and Euler\n"

        << "  " << programName
        << " calib    Run calibration and save profile\n";
}

int main(int argc, char *argv[])
{
    RunMode mode = RunMode::Euler;

    if (argc == 1)
    {
        mode = RunMode::Euler;
    }
    else if (argc == 2)
    {
        const std::string argument = argv[1];

        if (argument == "calib")
        {
            mode = RunMode::Calibration;
        }
        else if (argument == "info")
        {
            mode = RunMode::Info;
        }
        else if (argument == "all")
        {
            mode = RunMode::All;
        }
        else
        {
            printUsage(argv[0]);
            return 1;
        }
    }
    else
    {
        printUsage(argv[0]);
        return 1;
    }

    BNO055 bno;

    if (!bno.initialize(std::chrono::milliseconds(1000)))
    {
        std::cerr << "BNO055 init failed: "
                  << bno.lastError()
                  << '\n';

        return 1;
    }

    std::cout << "BNO055 initialized at address 0x"
              << std::hex
              << static_cast<unsigned>(bno.address())
              << std::dec
              << "\n\n";

    if (mode == RunMode::Calibration)
    {
        runCalibrationMode(bno);
    }
    else
    {
        runDisplayMode(bno, mode);
    }

    return 0;
}
