#pragma once

#include "common/Types.h"
#include <atomic>
#include <array>
#include <cstdint>
#include <string>
#include <opencv2/core.hpp>

namespace common
{
    /**
     * TelemetryClient: Client truyền hình ảnh Camera, thông số điều khiển (Tốc độ, Góc lái)
     * và Trạng thái nhận diện của xe từ Jetson Nano về Web Server thông qua UDP Datagram tốc độ cao.
     */
    class TelemetryClient
    {
    public:
        TelemetryClient(std::string serverIp, int port = 9999, std::string carId = "CAR_01");
        ~TelemetryClient();

        bool Init();
        void Close();
        bool IsInitialized() const { return initialized_; }

        // Đặt trạng thái nhận diện (0: None, 1..13: traffic signs / events)
        void SetVehicleState(int state);
        int GetVehicleState() const;
        void SetVehicleStates(const std::array<int, 3> &states);
        std::array<int, 3> GetVehicleStates() const;

        /**
         * Gửi ngay lập tức trạng thái nhận diện mới lên server (gói tin UDP siêu nhẹ,
         * gửi tức thì mà không cần chờ khung hình camera tiếp theo).
         */
        bool SendVehicleState(int state);
        bool SendVehicleStates(const std::array<int, 3> &states);

        /**
         * Gửi đồng thời khung hình camera (nén JPEG) kèm Tốc độ, Góc lái và Trạng thái nhận diện
         * trong một gói tin UDP nhị phân duy nhất (Low-latency, Zero-buffer).
         * 
         * Packet format (48-byte header + JPEG body):
         * [0..3]:   "UCR3" magic
         * [4..19]:  car_id (16 bytes, null-padded, hỗ trợ car_id dài tới 15 ký tự)
         * [20..23]: sequence (uint32_t, little-endian)
         * [24..27]: speed (float, cm/s, little-endian)
         * [28..31]: steering (int32_t, deg, little-endian)
         * [32..43]: vehicleStates[3] (3 x int32_t)
         * [44..47]: payloadSize (uint32_t)
         * [48..N]:  JPEG bytes
         */
        bool SendFrameAndTelemetry(const cv::Mat &frame,
                                   float speedCmPerSec,
                                   int steeringDeg,
                                   int vehicleState = -1,
                                   int width = 480,
                                   int height = 288,
                                   int quality = 55);

        bool SendFrameAndTelemetry(const cv::Mat &frame,
                                   float speedCmPerSec,
                                   int steeringDeg,
                                   const std::array<int, 3> &vehicleStates,
                                   int width = 480,
                                   int height = 288,
                                   int quality = 55);

    private:
        std::string serverIp_;
        int port_{9999};
        std::string carId_{"CAR_01"};
        int socket_{-1};
        bool initialized_{false};
        // Detection and video telemetry send from different worker threads.
        std::atomic<uint32_t> seq_{0};
        std::array<std::atomic<int>, 3> states_{{0, 0, 0}};
    };
} // namespace common
