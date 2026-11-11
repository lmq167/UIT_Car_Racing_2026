#pragma once

#include "common/Types.h"
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

        // Đặt trạng thái nhận diện (0: None, 1..12: traffic signs / events)
        void SetVehicleState(int state) { state_ = state; }
        int GetVehicleState() const { return state_; }

        /**
         * Gửi ngay lập tức trạng thái nhận diện mới lên server (gói tin UDP siêu nhẹ,
         * gửi tức thì mà không cần chờ khung hình camera tiếp theo).
         */
        bool SendVehicleState(int state);

        /**
         * Gửi đồng thời khung hình camera (nén JPEG) kèm Tốc độ, Góc lái và Trạng thái nhận diện
         * trong một gói tin UDP nhị phân duy nhất (Low-latency, Zero-buffer).
         * 
         * Cấu trúc gói (Packet Header 40 bytes + JPEG Body):
         * [0..3]:   "UCRC" (Magic Header 4 bytes)
         * [4..19]:  car_id (16 bytes, null-padded, hỗ trợ car_id dài tới 15 ký tự)
         * [20..23]: sequence (uint32_t, little-endian)
         * [24..27]: speed (float, cm/s, little-endian)
         * [28..31]: steering (int32_t, deg, little-endian)
         * [32..35]: vehicleState (int32_t, mã trạng thái nhận diện 0..12)
         * [36..39]: payloadSize (uint32_t, độ dài JPEG body)
         * [40..N]:  JPEG bytes
         */
        bool SendFrameAndTelemetry(const cv::Mat &frame,
                                   float speedCmPerSec,
                                   int steeringDeg,
                                   int vehicleState = -1,
                                   int width = 480,
                                   int height = 288,
                                   int quality = 55);

    private:
        std::string serverIp_;
        int port_{9999};
        std::string carId_{"CAR_01"};
        int socket_{-1};
        bool initialized_{false};
        uint32_t seq_{0};
        int state_{0};
    };
} // namespace common

