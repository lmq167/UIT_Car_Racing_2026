#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <string>

namespace perception {
struct InfoJson {
    cv::Size source, input;
    cv::Rect crop;
    std::string input_name, output_name;
    double scale;
    int num_classes, interpolation;
    std::array<float,3> mean, std;
    explicit InfoJson(const std::string &path, bool allowFakeData=false);
    size_t InputElements() const { return size_t(input.height)*input.width*3; }
    size_t OutputElements() const { return size_t(input.height)*input.width*num_classes; }
    size_t Elements() const { return OutputElements(); }
};
}
