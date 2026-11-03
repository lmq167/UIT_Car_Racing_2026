#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace control {
// P2: hộp trên ảnh gốc, thời điểm cùng đồng hồ đơn điệu với mặt nạ.
struct Detection {
    std::string class_name;
    cv::Rect2f box;
    float confidence=0;
    int64_t captured_ms=0;
};
struct Detections {
    std::vector<Detection> items;
    int64_t captured_ms=0;
};
}
