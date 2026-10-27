#include "hardware/Motor.h"
#include "common/Logger.h"

#include <atomic>
#include <cstdio>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <poll.h>

namespace
{
void LogUart(const char *direction, const char *data, std::size_t size)
{
    static const char hex[] = "0123456789ABCDEF";
    std::string escaped;
    for (std::size_t i = 0; i < size; ++i)
    {
        const unsigned char c = static_cast<unsigned char>(data[i]);
        if (c == '\r') escaped += "\\r";
        else if (c == '\n') escaped += "\\n";
        else if (c == '\\') escaped += "\\\\";
        else if (c == '"') escaped += "\\\"";
        else if (c >= 32 && c <= 126) escaped += static_cast<char>(c);
        else
        {
            escaped += "\\x";
            escaped += hex[c >> 4];
            escaped += hex[c & 15];
        }
    }
    std::cout << "[Motor UART " << direction << "] " << size
              << " bytes: \"" << escaped << "\"" << std::endl;
}
}

Motor::~Motor()
{
    if (_device != -1)
    {
        close(_device);
    }
}

bool Motor::Init()
{
    // Cờ toàn tiến trình: không thử lại chuỗi ghi cấu hình, kể cả sau lỗi.
    static std::atomic<bool> attempted{false};
    if (attempted.exchange(true)) throw std::runtime_error("Motor::Init da duoc goi");
    if (!OpenPortOnly()) return false;
    SendInitCommand();
    return true;
}

bool Motor::OpenPortOnly()
{
    if (_device < 0) _device = open(_device_port, O_RDWR | O_NOCTTY);
    if (_device == -1)
    {
        common::Logger::Error("Motor", "open",
                              std::string("cannot open ") + _device_port +
                                  ": " + std::strerror(errno));
        return false;
    }

    termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(_device, &tty) != 0)
    {
        common::Logger::Error("Motor", "tcgetattr", std::strerror(errno));
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(_device, TCSANOW, &tty) != 0)
    {
        common::Logger::Error("Motor", "tcsetattr", std::strerror(errno));
        return false;
    }

    if (_uartLogging)
        std::cout << "[Motor UART] port=" << _device_port
                  << " baud=115200 8N1 flow_control=none\n";
    return true;
}

void Motor::SendCommand(const char *cmd, bool IsReadRes)
{
    lastWriteOk_ = false;
    if (_device < 0)
    {
        common::Logger::Error("Motor", "uart_write", "device is not open");
        return;
    }
    if (cmd == nullptr)
    {
        common::Logger::Error("Motor", "uart_write", "null command");
        return;
    }

    const size_t length = std::strlen(cmd);
    const ssize_t written = write(_device, cmd, length);
    if (_uartLogging && written >= 0)
        LogUart("TX", cmd, static_cast<std::size_t>(written));
    if (written < 0)
    {
        common::Logger::Error("Motor", "uart_write", std::strerror(errno));
        return;
    }
    if (static_cast<size_t>(written) != length)
    {
        common::Logger::Error("Motor", "uart_write", "short write");
        return;
    }
    lastWriteOk_ = true;
    if (IsReadRes)
    {
        ReadDriverResponse();
    }
}

void Motor::ReadDriverResponse()
{
    usleep(10000);

    char readBuf[256];
    std::string response;
    int bytesRead = 0;

    while ((bytesRead = read(_device, readBuf, sizeof(readBuf) - 1)) > 0)
    {
        if (_uartLogging)
            LogUart("RX", readBuf, static_cast<std::size_t>(bytesRead));
        response.append(readBuf, static_cast<std::size_t>(bytesRead));
    }

    if (bytesRead < 0)
    {
        common::Logger::Error("Motor", "uart_read", std::strerror(errno));
        return;
    }

    if (_uartLogging && response.empty())
        std::cout << "[Motor UART RX] no bytes received for command\n";

    if (!_uartLogging && !response.empty())
    {
        std::cout << "[Driver -> MCU]: " << response;
        if (response.back() != '\n')
        {
            std::cout << '\n';
        }
    }
}

void Motor::SaveAndResetDriver()
{
    SendCommand("N1 $101=1\n");
    usleep(1000000);
}

void Motor::SendInitCommand()
{
    static std::atomic<bool> sent{false};
    if (sent.exchange(true)) throw std::runtime_error("Khong duoc lap khoi tao motor");
    SendCommand("N1 O K1\n");
    usleep(20000);
    SendCommand("N1 O D1\n");
    usleep(20000);
    SendCommand("N1 $008=0\n");
    const std::string cmd = std::string("N1 $002=") + std::to_string(PulsePerRound) + "\n";
    SendCommand(cmd.c_str());

    SetMode(1);
    SaveAndResetDriver();
    SendCommand("N1 O U C r\n");
    usleep(500000);
}

void Motor::SetMode(uint8_t mode)
{
    if (mode != 1 && mode != 2)
    {
        common::Logger::Warn("Motor", "set_mode", "invalid mode");
        return;
    }

    const std::string cmd = "N1 O M" + std::to_string(mode + 2) + "\n";
    SendCommand(cmd.c_str());
}

void Motor::SetSpeedRad(float Rad)
{
    const std::string cmd = "N1 v" + std::to_string(Rad) + "\n";
    SendCommand(cmd.c_str());
}

void Motor::SetSpeedCms(float Cms)
{
    const std::string cmd = "N1 v" + std::to_string((Cms * 2.0f) / D) + "\n";
    SendCommand(cmd.c_str(), false);
}

void Motor::PositionControl(bool enabled)
{
    SendCommand(enabled ? "N1 O M4\n" : "N1 O M3\n", false);
}

void Motor::SetSpeedPositionRad(float Position, float Rad)
{
    const std::string cmd =
        "N1 p" + std::to_string(Position) + " v" + std::to_string(Rad) + "\n";
    SendCommand(cmd.c_str());
}

void Motor::SetSpeedPositionCms(float PositionCm, float Cms)
{
    const std::string cmd =
        "N1 p" + std::to_string((PositionCm * 2.0f) / D) +
        " v" + std::to_string((Cms * 2.0f) / D) + "\n";
    SendCommand(cmd.c_str(), false);
}

bool Motor::GetStatus(char *buffer, std::size_t bufferSize)
{
    if (buffer == nullptr || bufferSize < 2 || _device < 0)
    {
        return false;
    }

    buffer[0] = '\0';

    // Xóa dữ liệu RX cũ.
    if (::tcflush(_device, TCIFLUSH) != 0)
    {
        common::Logger::Error(
            "Motor",
            "tcflush",
            std::strerror(errno));
        return false;
    }

    SendCommand("N1 O G1\n", false);

    char rawBuffer[512];
    std::size_t totalBytes = 0;

    constexpr int idleTimeoutMs = 20;
    while (totalBytes < sizeof(rawBuffer) - 1)
    {
        pollfd uart{_device, POLLIN, 0};
        const int ready = ::poll(&uart, 1, idleTimeoutMs);
        if (ready == 0)
        {
            if (_uartLogging && totalBytes == 0)
                std::cout << "[Motor UART RX] timeout: no bytes received for N1 O G1 within "
                          << idleTimeoutMs << " ms\n";
            break;
        }
        if (ready < 0)
        {
            if (errno == EINTR)
                continue;
            common::Logger::Error("Motor", "uart_poll", std::strerror(errno));
            return false;
        }
        if (uart.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            common::Logger::Error("Motor", "uart_poll", "UART disconnected or unavailable");
            return false;
        }
        if (!(uart.revents & POLLIN))
            continue;

        const ssize_t bytesRead = ::read(_device, rawBuffer + totalBytes, sizeof(rawBuffer) - 1 - totalBytes);

        if (bytesRead > 0)
        {
            if (_uartLogging)
                LogUart("RX", rawBuffer + totalBytes, static_cast<std::size_t>(bytesRead));
            totalBytes += static_cast<std::size_t>(bytesRead);
            continue;
        }

        if (bytesRead == 0)
        {
            break;
        }

        if (errno == EINTR)
        {
            continue;
        }

        common::Logger::Error(
            "Motor",
            "uart_read",
            std::strerror(errno));
        return false;
    }

    rawBuffer[totalBytes] = '\0';

    // Tìm dòng status thay vì giả định nó luôn là dòng thứ hai.
    const char *statusStart = std::strstr(rawBuffer, "N1 p");

    if (statusStart == nullptr)
    {
        common::Logger::Warn("Motor", "get_status", totalBytes == 0
            ? "no UART response to N1 O G1"
            : "response does not contain expected status N1 p; enable UART logging to inspect RX");
        return false;
    }

    // Tìm cuối dòng status.
    const char *statusEnd = std::strchr(statusStart, '\n');

    std::size_t statusLength = 0;

    if (statusEnd != nullptr)
    {
        statusLength =
            static_cast<std::size_t>(statusEnd - statusStart);
    }
    else
    {
        statusLength = std::strlen(statusStart);
    }

    // Bỏ CR nếu response kết thúc bằng "\r\n".
    while (statusLength > 0 &&
           (statusStart[statusLength - 1] == '\r' ||
            statusStart[statusLength - 1] == ' '))
    {
        --statusLength;
    }

    if (statusLength >= bufferSize)
    {
        common::Logger::Warn("Motor", "get_status", "status exceeds output buffer");
        return false;
    }

    std::memcpy(buffer, statusStart, statusLength);
    buffer[statusLength] = '\0';
    return true;
}

bool Motor::GetPositionCm(float *positionCm)
{
    if (positionCm == nullptr)
    {
        common::Logger::Error("Motor", "get_position", "null position pointer");
        return false;
    }

    char response[256];

    if (!GetStatus(response, sizeof(response)))
    {
        return false;
    }

    int address = 0;
    float positionRad = 0.0f;

    if (std::sscanf(response, "N%d p%f", &address, &positionRad) != 2)
    {
        common::Logger::Warn("Motor", "get_position", "cannot parse position from status");
        return false;
    }

    constexpr float wheelRadiusCm = 6.5f / 2.0f;

    *positionCm = positionRad * wheelRadiusCm;

    return true;
}
