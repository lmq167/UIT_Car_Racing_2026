#include "perception/RoadSegmenter.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace perception {
cv::Mat RoadSegmenter::Preprocess(const cv::Mat &image) const {
    if(image.type()!=CV_8UC3 || image.size()!=info_.source)
        throw std::runtime_error("Ảnh phải là BGR uint8 đúng source_image_hw");
    cv::Mat resized, rgb, scaled;
    cv::resize(image(info_.crop),resized,info_.input,0,0,cv::INTER_AREA);
    cv::cvtColor(resized,rgb,cv::COLOR_BGR2RGB);
    rgb.convertTo(scaled,CV_32F,info_.scale);
    const int dims[]={1,3,info_.input.height,info_.input.width};
    cv::Mat blob(4,dims,CV_32F);
    const size_t plane=blob.total()/3;
    std::vector<cv::Mat> channels;
    for(int c=0;c<3;++c) channels.emplace_back(info_.input,CV_32F,blob.ptr<float>()+c*plane);
    cv::split(scaled,channels);
    return blob;
}

cv::Mat RoadSegmenter::Decode(const float *logits) const {
    if(!logits) throw std::runtime_error("Thiếu logits");
    cv::Mat mask(info_.input,CV_8U);
    const size_t plane=mask.total();
    for(size_t i=0;i<plane;++i) {
        int best=0;
        for(int c=0;c<3;++c) {
            if(!std::isfinite(logits[c*plane+i])) throw std::runtime_error("Logits chứa NaN/Inf");
            if(logits[c*plane+i]>logits[best*plane+i]) best=c;
        }
        mask.ptr<unsigned char>()[i]=static_cast<unsigned char>(best);
    }
    return mask;
}

cv::Mat RoadSegmenter::Restore(const cv::Mat &mask) const {
    if(mask.type()!=CV_8UC1 || mask.size()!=info_.input)
        throw std::runtime_error("Mask phải là CV_8U đúng input_hw");
    cv::Mat full(info_.source,CV_8U,cv::Scalar(0)), resized;
    cv::resize(mask,resized,info_.crop.size(),0,0,cv::INTER_NEAREST);
    resized.copyTo(full(info_.crop));
    return full;
}

cv::Mat RoadSegmenter::Infer(const cv::Mat &image, IInferEngine &engine) const {
    const auto blob=Preprocess(image);
    std::vector<float> logits(info_.Elements());
    engine.Infer(blob.ptr<float>(),logits.data());
    return Decode(logits.data());
}
}
