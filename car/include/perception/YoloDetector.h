#pragma once
#include "perception/TrtEngine.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace perception {
struct YoloInfo {
    cv::Size input, resized;
    double letterbox_scale;
    int pad_left, pad_top, pad_right, pad_bottom;
    int candidates, num_classes;
    float conf_threshold, nms_iou;
    std::string input_name, output_name;
    std::vector<std::string> class_names;
    explicit YoloInfo(const std::string &path);
    size_t OutputElements() const { return size_t(4+num_classes)*candidates; }
};

struct Detection {
    std::string class_name;
    cv::Rect2f box; // Toạ độ ảnh gốc 1280×720, đã kẹp biên.
    float confidence;
};

class YoloDetector {
public:
    explicit YoloDetector(const YoloInfo &info) : info_(info) {}
    cv::Mat Preprocess(const cv::Mat &bgr1280x720) const;
    // out trỏ tới ít nhất info.OutputElements() số float, bố trí [1,4+num_classes,N].
    std::vector<Detection> Decode(const float *out) const;
    std::vector<Detection> Infer(const cv::Mat &image, IInferEngine &engine) const;
private:
    YoloInfo info_;
};
}
