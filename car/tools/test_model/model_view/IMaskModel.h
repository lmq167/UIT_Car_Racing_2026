#pragma once
#include "ModelInfo.h"
#include <opencv2/core.hpp>

namespace model_view {
// Giao diện suy luận; chỉ trả mặt nạ để hiển thị, không biết gì về motor/servo.
class IMaskModel {
public:
    virtual ~IMaskModel() = default;
    virtual const ModelInfo& info() const = 0;
    // Vào: ảnh BGR nguyên khung camera. Ra: CV_8UC1 cỡ input_hw, giá trị là chỉ số lớp.
    virtual cv::Mat infer(const cv::Mat& bgr) = 0;
};
} // namespace model_view
