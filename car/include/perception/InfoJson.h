#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace perception {
struct InfoJson {
    cv::Size source, input;
    cv::Rect crop;
    std::string input_name, output_name;
    double scale;
    explicit InfoJson(const std::string &path);
    size_t Elements() const { return size_t(input.height)*input.width*3; }
};
}
