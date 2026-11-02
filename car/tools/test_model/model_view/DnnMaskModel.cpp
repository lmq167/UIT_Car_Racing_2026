#include "DnnMaskModel.h"
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace model_view {
cv::Mat makeInputBlob(const cv::Mat& bgr, const ModelInfo& info) {
    if (bgr.type() != CV_8UC3 || bgr.rows != info.sourceH || bgr.cols != info.sourceW)
        throw std::runtime_error("Anh camera khac source_image_hw cua model");
    const cv::Rect crop(info.cropX0, info.cropY0, info.cropX1 - info.cropX0, info.cropY1 - info.cropY0);
    cv::Mat small;
    cv::resize(bgr(crop), small, cv::Size(info.inputW, info.inputH), 0, 0, cv::INTER_AREA);
    return cv::dnn::blobFromImage(small, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
}

cv::Mat argmaxMask(const cv::Mat& logits, const ModelInfo& info) {
    const int classes = static_cast<int>(info.classNames.size());
    if (logits.dims != 4 || logits.type() != CV_32F || logits.size[0] != 1 || logits.size[1] != classes ||
        logits.size[2] != info.inputH || logits.size[3] != info.inputW)
        throw std::runtime_error("Dau ra model khac output_shape");
    cv::Mat mask(info.inputH, info.inputW, CV_8UC1);
    const auto* data = logits.ptr<float>();
    const size_t pixels = mask.total();
    for (size_t i = 0; i < pixels; ++i) {
        int best = 0;
        for (int c = 1; c < classes; ++c)
            if (data[size_t(c) * pixels + i] > data[size_t(best) * pixels + i]) best = c;
        mask.data[i] = static_cast<unsigned char>(best);
    }
    return mask;
}

DnnMaskModel::DnnMaskModel(const std::string& onnxPath, const ModelInfo& info)
    : info_(info), net_(cv::dnn::readNetFromONNX(onnxPath)) {
    if (net_.empty()) throw std::runtime_error("Khong doc duoc ONNX: " + onnxPath);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

cv::Mat DnnMaskModel::infer(const cv::Mat& bgr) {
    net_.setInput(makeInputBlob(bgr, info_));
    return argmaxMask(net_.forward(), info_);
}
} // namespace model_view
