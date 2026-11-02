#pragma once
#include "IMaskModel.h"
#include <opencv2/dnn.hpp>
#include <string>

namespace model_view {
// Thuần: cắt vùng theo info, INTER_AREA về input_hw, BGR→RGB, /255, NCHW float32.
cv::Mat makeInputBlob(const cv::Mat& bgr, const ModelInfo& info);
// Thuần: logits [1,C,H,W] → CV_8UC1 argmax theo kênh.
cv::Mat argmaxMask(const cv::Mat& logits, const ModelInfo& info);

class DnnMaskModel : public IMaskModel {
public:
    DnnMaskModel(const std::string& onnxPath, const ModelInfo& info);
    const ModelInfo& info() const override { return info_; }
    cv::Mat infer(const cv::Mat& bgr) override;
private:
    ModelInfo info_;
    cv::dnn::Net net_;
};
} // namespace model_view
