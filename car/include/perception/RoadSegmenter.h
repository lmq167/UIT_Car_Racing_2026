#pragma once
#include "perception/InfoJson.h"
#include "perception/TrtEngine.h"

namespace perception {
class RoadSegmenter {
public:
    explicit RoadSegmenter(const InfoJson &info) : info_(info) {}
    cv::Mat Preprocess(const cv::Mat &bgr1280x720) const;
    cv::Mat Decode(const float *logits) const;
    cv::Mat Restore(const cv::Mat &mask) const;
    cv::Mat Infer(const cv::Mat &image, IInferEngine &engine) const;
private:
    InfoJson info_;
};
}
