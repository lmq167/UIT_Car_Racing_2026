#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "common/TelemetryClient.h"
#include <opencv2/imgcodecs.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void Check(bool ok, const std::string &message)
{
    if (!ok) throw std::runtime_error(message);
}

class Receiver {
public:
    Receiver()
    {
#ifdef _WIN32
        WSADATA data{};
        Check(WSAStartup(MAKEWORD(2, 2), &data) == 0, "Không khởi tạo được Winsock");
#endif
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        Check(socket_ != InvalidSocket, "Không tạo được bộ nhận UDP");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        Check(bind(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0,
              "Không gắn được UDP vào 127.0.0.1");
#ifdef _WIN32
        int length = sizeof(address);
#else
        socklen_t length = sizeof(address);
#endif
        Check(getsockname(socket_, reinterpret_cast<sockaddr *>(&address), &length) == 0,
              "Không đọc được cổng UDP");
        port_ = ntohs(address.sin_port);
    }

    ~Receiver()
    {
#ifdef _WIN32
        if (socket_ != InvalidSocket) closesocket(socket_);
        WSACleanup();
#else
        if (socket_ != InvalidSocket) close(socket_);
#endif
    }

    int Port() const { return port_; }

    std::vector<uint8_t> Receive()
    {
        fd_set ready;
        FD_ZERO(&ready);
        FD_SET(socket_, &ready);
        timeval timeout{2, 0};
        Check(select(static_cast<int>(socket_) + 1, &ready, nullptr, nullptr, &timeout) == 1,
              "Không nhận được gói UDP trong 2 giây");
        std::vector<uint8_t> packet(65536);
        const int count = recvfrom(socket_, reinterpret_cast<char *>(packet.data()),
                                   static_cast<int>(packet.size()), 0, nullptr, nullptr);
        Check(count >= 0, "Lỗi nhận gói UDP");
        packet.resize(static_cast<std::size_t>(count));
        return packet;
    }

private:
#ifdef _WIN32
    static constexpr SOCKET InvalidSocket = INVALID_SOCKET;
    SOCKET socket_{InvalidSocket};
#else
    static constexpr int InvalidSocket = -1;
    int socket_{InvalidSocket};
#endif
    int port_{0};
};

uint32_t ReadU32(const std::vector<uint8_t> &packet, std::size_t offset)
{
    return uint32_t(packet.at(offset)) |
           (uint32_t(packet.at(offset + 1)) << 8) |
           (uint32_t(packet.at(offset + 2)) << 16) |
           (uint32_t(packet.at(offset + 3)) << 24);
}

std::vector<uint8_t> CheckPacket(const std::vector<uint8_t> &packet,
                               const std::string &carId, uint32_t sequence,
                               float speed, int32_t steering,
                               const std::array<int, 3> &states)
{
    Check(packet.size() >= 48, "Gói UCR3 phải có đầu gói đủ 48 byte");
    Check(std::memcmp(packet.data(), "UCR3", 4) == 0, "Sai magic UCR3");
    char expectedId[16]{};
    std::strncpy(expectedId, carId.c_str(), sizeof(expectedId) - 1);
    Check(std::memcmp(packet.data() + 4, expectedId, sizeof(expectedId)) == 0,
          "car_id phải gồm 16 byte, tối đa 15 ký tự và đệm null");
    Check(ReadU32(packet, 20) == sequence, "Sai thứ tự hoặc byte little-endian của sequence");
    const uint32_t speedBits = ReadU32(packet, 24);
    float receivedSpeed = 0;
    static_assert(sizeof(receivedSpeed) == sizeof(speedBits), "float phải dài 32 bit");
    std::memcpy(&receivedSpeed, &speedBits, sizeof(receivedSpeed));
    Check(receivedSpeed == speed, "Sai float little-endian hoặc đơn vị cm/s");
    Check(ReadU32(packet, 28) == static_cast<uint32_t>(steering),
          "Sai steering int32 little-endian");
    for (std::size_t i = 0; i < states.size(); ++i)
        Check(ReadU32(packet, 32 + i * 4) == static_cast<uint32_t>(states[i]),
              "Sai states[" + std::to_string(i) + "] int32 little-endian");
    Check(ReadU32(packet, 44) == packet.size() - 48,
          "Sai payloadSize uint32 hoặc vị trí bắt đầu JPEG ở byte 48");
    return {packet.begin() + 48, packet.end()};
}

void CheckJpeg(const std::vector<uint8_t> &jpeg, int width, int height)
{
    Check(jpeg.size() >= 4 && jpeg[0] == 0xff && jpeg[1] == 0xd8 &&
          jpeg[jpeg.size() - 2] == 0xff && jpeg.back() == 0xd9,
          "Payload phải là JPEG đầy đủ");
    const cv::Mat image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    Check(!image.empty() && image.cols == width && image.rows == height,
          "JPEG phải giải mã được và có đúng kích thước yêu cầu");
    const cv::Vec3b pixel = image.at<cv::Vec3b>(height / 2, width / 2);
    Check(cv::norm(pixel, cv::Vec3b(20, 80, 160), cv::NORM_INF) <= 5,
          "Nội dung JPEG phải giữ màu khung hình đầu vào");
}

void CheckVehicleStates()
{
    using State = common::VehicleState;
    const std::array<State, 14> states{{
        State::NONE, State::GREEN_LIGHT, State::RED_LIGHT, State::ROAD_WORK,
        State::BRIDGE, State::ROUNDABOUT, State::OBSTACLE, State::TURN_RIGHT,
        State::NO_TURN_RIGHT, State::TURN_LEFT, State::NO_TURN_LEFT,
        State::PARKING_SIGN, State::PARKING, State::PEDESTRIAN
    }};
    for (std::size_t i = 0; i < states.size(); ++i)
        Check(static_cast<int>(states[i]) == static_cast<int>(i),
              "VehicleState phải đúng mã BTC 0–13");
}
}

int main()
{
    try
    {
        CheckVehicleStates();
        Receiver receiver;
        const std::string carId = "CAR6";
        common::TelemetryClient client("127.0.0.1", receiver.Port(), carId);
        Check(!client.IsInitialized(), "Client mới chưa được khởi tạo");
        Check(!client.SendVehicleState(13), "Chưa Init không được gửi trạng thái");
        Check(client.GetVehicleState() == 13, "API cũ phải giữ trạng thái trước Init");
        Check(client.Init() && client.IsInitialized(), "Không khởi tạo được client telemetry");
        uint32_t sequence = 1;
        Check(client.SendVehicleState(13), "Không gửi được trạng thái bằng API cũ");
        Check(CheckPacket(receiver.Receive(), carId, sequence++, 0, 0, {{13, 0, 0}}).empty(),
              "Gói chỉ trạng thái phải dài đúng 48 byte, không có payload");

        const cv::Mat frame(24, 32, CV_8UC3, cv::Scalar(20, 80, 160));
        Check(!client.SendFrameAndTelemetry(cv::Mat(), 1.25f, 0), "Ảnh rỗng không được gửi");
        client.SetVehicleState(4);
        Check(client.GetVehicleState() == 4, "Set/GetVehicleState phải giữ API cũ");
        Check(client.SendFrameAndTelemetry(frame, 123.25f, -17, -1, 64, 48, 55),
              "Không gửi được ảnh dùng trạng thái đã lưu");
        CheckJpeg(CheckPacket(receiver.Receive(), carId, sequence++, 123.25f, -17,
                              {{4, 0, 0}}), 64, 48);
        Check(client.SendFrameAndTelemetry(frame, -12.5f, 27, 8, 32, 24, 70),
              "Không gửi được ảnh với tham số một trạng thái");
        CheckJpeg(CheckPacket(receiver.Receive(), carId, sequence++, -12.5f, 27,
                              {{8, 0, 0}}), 32, 24);

// Chỉ bỏ API ba trạng thái khi biên dịch kiểm ngược với bản UCRC cũ.
#ifndef UCR_TELEMETRY_LEGACY_API_ONLY
        const std::array<int, 3> detected{{1, 6, 13}};
        client.SetVehicleStates(detected);
        Check(client.GetVehicleStates() == detected && client.GetVehicleState() == 1,
              "Set/GetVehicleStates phải giữ đủ ba trạng thái");
        Check(client.SendFrameAndTelemetry(frame, 25.5f, -19),
              "API ảnh cũ phải dùng đủ ba trạng thái đang lưu");
        CheckJpeg(CheckPacket(receiver.Receive(), carId, sequence++, 25.5f, -19, detected), 480, 288);
        const std::array<int, 3> frameStates{{2, 10, 11}};
        Check(client.SendFrameAndTelemetry(frame, -5.5f, 24, frameStates, 64, 48, 60),
              "Không gửi được ảnh bằng API ba trạng thái");
        CheckJpeg(CheckPacket(receiver.Receive(), carId, sequence++, -5.5f, 24, frameStates), 64, 48);
        Check(client.GetVehicleStates() == frameStates, "API ảnh phải cập nhật ba trạng thái");
        const std::array<int, 3> eventStates{{3, 5, 12}};
        Check(client.SendVehicleStates(eventStates), "Không gửi được gói ba trạng thái");
        Check(CheckPacket(receiver.Receive(), carId, sequence++, 0, 0, eventStates).empty(),
              "Gói ba trạng thái phải có payloadSize bằng 0");
        client.SetVehicleState(9);
        Check(client.GetVehicleStates() == std::array<int, 3>{{9, 0, 0}},
              "SetVehicleState phải xóa hai trạng thái phụ");
        client.SetVehicleStates(eventStates);
        Check(client.SendVehicleState(7), "Không gửi lại được API một trạng thái");
        Check(CheckPacket(receiver.Receive(), carId, sequence++, 0, 0, {{7, 0, 0}}).empty(),
              "SendVehicleState phải xóa hai trạng thái phụ");
        client.SetVehicleStates(eventStates);
        Check(client.SendFrameAndTelemetry(frame, 3.5f, -8, 6, 32, 24, 55),
              "Không gửi lại được ảnh bằng API một trạng thái");
        CheckJpeg(CheckPacket(receiver.Receive(), carId, sequence++, 3.5f, -8,
                              {{6, 0, 0}}), 32, 24);
#endif

        client.Close();
        Check(!client.IsInitialized() && !client.SendVehicleState(1),
              "Sau Close không được gửi gói UDP");
        const std::string longId = "CAR_1234567890123456789";
        common::TelemetryClient longIdClient("127.0.0.1", receiver.Port(), longId);
        Check(longIdClient.Init() && longIdClient.SendVehicleState(12),
              "Không gửi được gói có car_id dài");
        Check(CheckPacket(receiver.Receive(), longId, 1, 0, 0, {{12, 0, 0}}).empty(),
              "car_id dài không được làm lệch đầu gói");
        std::cout << "PASS: UCR3 48 byte, JPEG, ba trạng thái, API cũ và enum BTC 0–13\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
