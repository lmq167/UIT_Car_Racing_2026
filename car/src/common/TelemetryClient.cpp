#include "common/TelemetryClient.h"
#include "common/Logger.h"

#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using SocketHandle = SOCKET;
#define CLOSE_SOCKET closesocket
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
using SocketHandle = int;
#define CLOSE_SOCKET ::close
constexpr int INVALID_SOCKET = -1;
#endif

namespace common
{
TelemetryClient::TelemetryClient(std::string serverIp, int port, std::string carId)
    : serverIp_(std::move(serverIp)),
      port_(port),
      carId_(std::move(carId))
{
}

TelemetryClient::~TelemetryClient()
{
    Close();
}

bool TelemetryClient::Init()
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    socket_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, 0));

    if (socket_ < 0)
    {
        Logger::Error("Telemetry", "init", "Failed to create UDP socket");
        Close();
        return false;
    }

    // Tăng kích thước bộ đệm gửi (Send Buffer) lên 256KB để tránh nghẽn khi gửi frame video
    int sndBufSize = 256 * 1024;
    setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char *>(&sndBufSize), sizeof(sndBufSize));

    initialized_ = true;
    Logger::Info("Telemetry", "init",
                 "Telemetry UDP client ready -> Target Server: " + serverIp_ +
                     ":" + std::to_string(port_) + " [Car ID: " + carId_ + "]");
    return true;
}

void TelemetryClient::Close()
{
    if (socket_ >= 0)
    {
        CLOSE_SOCKET(socket_);
        socket_ = -1;
    }
#ifdef _WIN32
    WSACleanup();
#endif
    initialized_ = false;
}

void TelemetryClient::SetVehicleState(int state)
{
    SetVehicleStates({{state, 0, 0}});
}

int TelemetryClient::GetVehicleState() const
{
    return states_[0].load();
}

void TelemetryClient::SetVehicleStates(const std::array<int, 3> &states)
{
    for (std::size_t i = 0; i < states.size(); ++i) states_[i].store(states[i]);
}

std::array<int, 3> TelemetryClient::GetVehicleStates() const
{
    return {{states_[0].load(), states_[1].load(), states_[2].load()}};
}

bool TelemetryClient::SendVehicleState(int state)
{
    return SendVehicleStates({{state, 0, 0}});
}

bool TelemetryClient::SendVehicleStates(const std::array<int, 3> &states)
{
    SetVehicleStates(states);
    if (!initialized_ || socket_ < 0)
        return false;

    // Fast state-only packet (UCR3, 48-byte header, no image payload).
    constexpr size_t HEADER_SIZE = 48;
    std::vector<uint8_t> packet;
    packet.reserve(HEADER_SIZE);

    // Magic Header: "UCR3"
    packet.push_back('U');
    packet.push_back('C');
    packet.push_back('R');
    packet.push_back('3');

    // Car ID (16 bytes, null-padded)
    char idBuf[16] = {0};
    std::strncpy(idBuf, carId_.c_str(), sizeof(idBuf) - 1);
    packet.insert(packet.end(), idBuf, idBuf + 16);

    // Sequence (uint32_t)
    const uint32_t seq = ++seq_;
    const uint8_t *seqBytes = reinterpret_cast<const uint8_t *>(&seq);
    packet.insert(packet.end(), seqBytes, seqBytes + 4);

    // Speed: 0.0f
    const float speed = 0.0f;
    const uint8_t *speedBytes = reinterpret_cast<const uint8_t *>(&speed);
    packet.insert(packet.end(), speedBytes, speedBytes + 4);

    // Steering: 0
    const int32_t steeringVal = 0;
    const uint8_t *steeringBytes = reinterpret_cast<const uint8_t *>(&steeringVal);
    packet.insert(packet.end(), steeringBytes, steeringBytes + 4);

    // Vehicle State (int32_t)
    for (int stateValue : states)
    {
        const int32_t stateVal = static_cast<int32_t>(stateValue);
        const uint8_t *stateBytes = reinterpret_cast<const uint8_t *>(&stateVal);
        packet.insert(packet.end(), stateBytes, stateBytes + 4);
    }

    // Payload size: 0 (No image)
    const uint32_t payloadSize = 0;
    const uint8_t *sizeBytes = reinterpret_cast<const uint8_t *>(&payloadSize);
    packet.insert(packet.end(), sizeBytes, sizeBytes + 4);

    sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(static_cast<uint16_t>(port_));
    destAddr.sin_addr.s_addr = inet_addr(serverIp_.c_str());

    const int sent = sendto(socket_,
                            reinterpret_cast<const char *>(packet.data()),
                            static_cast<int>(packet.size()),
                            0,
                            reinterpret_cast<const sockaddr *>(&destAddr),
                            sizeof(destAddr));
    return sent > 0;
}

bool TelemetryClient::SendFrameAndTelemetry(const cv::Mat &frame,
                                           float speedCmPerSec,
                                           int steeringDeg,
                                           int vehicleState,
                                           int width,
                                           int height,
                                           int quality)
{
    const std::array<int, 3> states = vehicleState >= 0
        ? std::array<int, 3>{{vehicleState, 0, 0}}
        : GetVehicleStates();
    return SendFrameAndTelemetry(frame, speedCmPerSec, steeringDeg, states,
                                 width, height, quality);
}

bool TelemetryClient::SendFrameAndTelemetry(const cv::Mat &frame,
                                           float speedCmPerSec,
                                           int steeringDeg,
                                           const std::array<int, 3> &vehicleStates,
                                           int width,
                                           int height,
                                           int quality)
{
    SetVehicleStates(vehicleStates);

    if (!initialized_ || socket_ < 0 || frame.empty())
        return false;

    // 1. Resize khung hình nếu cần
    cv::Mat resized;
    if (frame.cols != width || frame.rows != height)
    {
        cv::resize(frame, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    }
    else
    {
        resized = frame;
    }

    // 2. Nén JPEG với chất lượng định trước (mặc định 55% ~ 12-18KB)
    std::vector<uchar> jpegBuffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(quality, 10, 95)};
    if (!cv::imencode(".jpg", resized, jpegBuffer, params))
    {
        return false;
    }

    // Kiểm tra kích thước gói không vượt quá giới hạn an toàn UDP (~64KB)
    if (jpegBuffer.size() > 60000)
    {
        Logger::Warn("Telemetry", "send", "Compressed frame size (" + std::to_string(jpegBuffer.size()) + " bytes) exceeds safe UDP limit");
        return false;
    }

    // 3. Build UCR3 packet: 48-byte header + JPEG body
    // [0..3]:   "UCR3" (Magic Header)
    // [4..19]:  car_id (16 bytes, null-padded)
    // [20..23]: sequence (uint32_t)
    // [24..27]: speedCmPerSec (float)
    // [28..31]: steeringDeg (int32_t)
    // [32..43]: vehicleStates[3] (3 x int32_t)
    // [44..47]: payloadSize (uint32_t)
    // [48..N]:  JPEG bytes
    constexpr size_t HEADER_SIZE = 48;
    std::vector<uint8_t> packet;
    packet.reserve(HEADER_SIZE + jpegBuffer.size());

    // Magic Header: "UCR3"
    packet.push_back('U');
    packet.push_back('C');
    packet.push_back('R');
    packet.push_back('3');

    // Car ID (16 bytes, null-padded)
    char idBuf[16] = {0};
    std::strncpy(idBuf, carId_.c_str(), sizeof(idBuf) - 1);
    packet.insert(packet.end(), idBuf, idBuf + 16);

    // Sequence (uint32_t)
    const uint32_t seq = ++seq_;
    const uint8_t *seqBytes = reinterpret_cast<const uint8_t *>(&seq);
    packet.insert(packet.end(), seqBytes, seqBytes + 4);

    // Speed (float)
    const uint8_t *speedBytes = reinterpret_cast<const uint8_t *>(&speedCmPerSec);
    packet.insert(packet.end(), speedBytes, speedBytes + 4);

    // Steering (int32_t)
    const int32_t steeringVal = static_cast<int32_t>(steeringDeg);
    const uint8_t *steeringBytes = reinterpret_cast<const uint8_t *>(&steeringVal);
    packet.insert(packet.end(), steeringBytes, steeringBytes + 4);

    // Vehicle State (int32_t)
    for (int stateValue : vehicleStates)
    {
        const int32_t stateVal = static_cast<int32_t>(stateValue);
        const uint8_t *stateBytes = reinterpret_cast<const uint8_t *>(&stateVal);
        packet.insert(packet.end(), stateBytes, stateBytes + 4);
    }

    // Payload size (uint32_t)
    const uint32_t payloadSize = static_cast<uint32_t>(jpegBuffer.size());
    const uint8_t *sizeBytes = reinterpret_cast<const uint8_t *>(&payloadSize);
    packet.insert(packet.end(), sizeBytes, sizeBytes + 4);

    // JPEG data bytes
    packet.insert(packet.end(), jpegBuffer.begin(), jpegBuffer.end());

    // 4. Gửi qua UDP Socket
    sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(static_cast<uint16_t>(port_));
    destAddr.sin_addr.s_addr = inet_addr(serverIp_.c_str());

    const int sent = sendto(socket_,
                            reinterpret_cast<const char *>(packet.data()),
                            static_cast<int>(packet.size()),
                            0,
                            reinterpret_cast<const sockaddr *>(&destAddr),
                            sizeof(destAddr));
    return sent > 0;
}

} // namespace common
