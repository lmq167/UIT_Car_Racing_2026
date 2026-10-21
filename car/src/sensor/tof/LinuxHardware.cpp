#include "LinuxHardware.h"
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <algorithm>
#include <vector>

namespace tof {
namespace {
void writeFile(const std::string& path, const std::string& value) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) throw std::system_error(errno, std::generic_category(), path);
    const auto written = ::write(fd, value.data(), value.size());
    const int saved = errno;
    ::close(fd);
    if (written != static_cast<ssize_t>(value.size()))
        throw std::system_error(written < 0 ? saved : EIO, std::generic_category(), path);
}
int gpioBase() {
    DIR* directory = ::opendir("/sys/class/gpio");
    if (!directory) throw std::system_error(errno, std::generic_category(), "/sys/class/gpio");
    int base = -1;
    while (const auto* entry = ::readdir(directory)) {
        const std::string name(entry->d_name);
        if (name.compare(0, 8, "gpiochip") != 0) continue;
        const std::string path = "/sys/class/gpio/" + name;
        std::ifstream labelFile(path + "/label");
        std::string label;
        std::getline(labelFile, label);
        if (label == "tegra-gpio") {
            std::ifstream baseFile(path + "/base");
            baseFile >> base;
            break;
        }
    }
    ::closedir(directory);
    if (base < 0) throw std::runtime_error("Cannot locate tegra-gpio sysfs chip (requires classic Nano/L4T GPIO sysfs)");
    return base;
}
void transfer(int fd, i2c_msg* messages, unsigned count) {
    i2c_rdwr_ioctl_data data{};
    data.msgs = messages;
    data.nmsgs = count;
    const int result = ::ioctl(fd, I2C_RDWR, &data);
    if (result < 0) throw std::system_error(errno, std::generic_category(), "I2C_RDWR");
    if (result != static_cast<int>(count)) throw std::runtime_error("Incomplete I2C transfer");
}
}

LinuxHardware::LinuxHardware(const std::string& device) {
    std::ifstream compatibleFile("/proc/device-tree/compatible", std::ios::binary);
    const std::string compatible((std::istreambuf_iterator<char>(compatibleFile)), {});
    if (compatible.find("nvidia,tegra210") == std::string::npos)
        throw std::runtime_error("This GPIO mapping requires Jetson Nano/Tegra210, not Orin Nano");
    gpioBase_ = gpioBase();
    fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), device);
    unsigned long functions = 0;
    if (::ioctl(fd_, I2C_FUNCS, &functions) < 0 || !(functions & I2C_FUNC_I2C)) {
        std::cerr << "[ERROR] [TOF] [I2C_FUNCS] Adapter does not provide raw I2C transfers; port kept open\n";
    }
}
LinuxHardware::~LinuxHardware() {
    if (fd_ >= 0) ::close(fd_);
    // Keep GPIOs exported and HIGH after successful setup so addresses survive.
}
void LinuxHardware::delayMs(unsigned ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void LinuxHardware::shutdown(std::size_t slot, bool enabled) {
    const int gpio = gpioBase_ + kSlots.at(slot).gpioOffset;
    const std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio);
    if (!prepared_.at(slot)) {
        if (::access(path.c_str(), F_OK) != 0)
            writeFile("/sys/class/gpio/export", std::to_string(gpio));
        for (unsigned i = 0; i < 100 && ::access((path + "/direction").c_str(), F_OK) != 0; ++i)
            delayMs(5);
        writeFile(path + "/active_low", "0");
        writeFile(path + "/direction", enabled ? "high" : "low");
        prepared_[slot] = true;
    } else {
        writeFile(path + "/value", enabled ? "1" : "0");
    }
}
uint8_t LinuxHardware::readByte(uint8_t address, uint16_t reg, bool wide) {
    uint8_t value = 0;
    readBlock(address, reg, wide, &value, 1);
    return value;
}
void LinuxHardware::readBlock(uint8_t address, uint16_t reg, bool wide, uint8_t* data, std::size_t count) {
    if (!data || count == 0 || count > 4096)
        throw std::invalid_argument("Invalid I2C read block");
    uint8_t pointer[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
    i2c_msg messages[2]{};
    messages[0].addr = address;
    messages[0].len = wide ? 2 : 1;
    messages[0].buf = wide ? pointer : pointer + 1;
    messages[1].addr = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(count);
    messages[1].buf = data;
    transfer(fd_, messages, 2);
}
void LinuxHardware::writeByte(uint8_t address, uint16_t reg, bool wide, uint8_t value) {
    writeBlock(address, reg, wide, &value, 1);
}
void LinuxHardware::writeBlock(uint8_t address, uint16_t reg, bool wide, const uint8_t* data, std::size_t count) {
    if (!data || count == 0 || count > 4094)
        throw std::invalid_argument("Invalid I2C write block");
    std::vector<uint8_t> bytes;
    if (wide) bytes.push_back(static_cast<uint8_t>(reg >> 8));
    bytes.push_back(static_cast<uint8_t>(reg));
    bytes.insert(bytes.end(), data, data + count);
    i2c_msg message{};
    message.addr = address;
    message.len = static_cast<__u16>(bytes.size());
    message.buf = bytes.data();
    transfer(fd_, &message, 1);
}
}
