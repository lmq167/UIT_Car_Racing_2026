#include "ModelInfo.h"
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>

namespace model_view {
namespace {
const char* const kCvdnnSuffix = "_cvdnn.onnx";
const char* const kOnnxSuffix = ".onnx";

[[noreturn]] void bad(const std::string& message) { throw std::runtime_error("_info.json: " + message); }
bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}
std::string text(const cv::FileNode& root, const char* key) {
    const auto node = root[key];
    if (!node.isString()) bad(std::string("thieu truong chuoi ") + key);
    return static_cast<std::string>(node);
}
void expect(const cv::FileNode& root, const char* key, const std::string& wanted) {
    const auto value = text(root, key);
    if (value != wanted) bad(std::string(key) + "=" + value + ", can " + wanted);
}
// JSON true/false của FileStorage là số nguyên 1/0.
bool flag(const cv::FileNode& root, const char* key) {
    const auto node = root[key];
    if (!node.isInt() || (static_cast<int>(node) != 0 && static_cast<int>(node) != 1))
        bad(std::string("thieu truong dung/sai ") + key);
    return static_cast<int>(node) == 1;
}
void pair(const cv::FileNode& root, const char* key, int& first, int& second) {
    const auto node = root[key];
    if (!node.isSeq() || node.size() != 2 || !node[0].isInt() || !node[1].isInt())
        bad(std::string("thieu cap so nguyen ") + key);
    first = static_cast<int>(node[0]);
    second = static_cast<int>(node[1]);
}
void shape(const cv::FileNode& root, const char* key, const int (&wanted)[4]) {
    const auto node = root[key];
    if (!node.isSeq() || node.size() != 4) bad(std::string("thieu ") + key);
    for (int i = 0; i < 4; ++i)
        if (!node[i].isInt() || static_cast<int>(node[i]) != wanted[i]) bad(std::string(key) + " sai");
}
} // namespace

ModelInfo loadModelInfo(const std::string& path) {
    cv::FileStorage file;
    try {
        if (!file.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON)) bad("khong mo duoc " + path);
    } catch (const cv::Exception&) { bad("JSON hong: " + path); }
    const auto root = file.root();
    ModelInfo info;
    expect(root, "task", "segmentation");
    expect(root, "output_type", "logits");
    expect(root, "activation", "softmax_over_channels_argmax");
    expect(root, "color_order", "RGB");
    expect(root, "layout", "NCHW");
    if (flag(root, "preprocessing_in_model")) bad("preprocessing_in_model phai la false");
    info.imagenetInModel = flag(root, "imagenet_normalization_in_model");
    info.fakeData = flag(root, "fake_data");
    info.name = text(root, "architecture");
    if (root["variant"].isString()) info.name = static_cast<std::string>(root["variant"]) + "/" + info.name;

    const auto classes = root["class_names"];
    if (!classes.isSeq() || classes.empty()) bad("thieu class_names");
    for (const auto& item : classes) {
        if (!item.isString()) bad("class_names phai la chuoi");
        info.classNames.push_back(static_cast<std::string>(item));
    }
    const std::vector<std::string> known{"background", "track_surface", "line"};
    if (info.classNames != known) bad("lop la, can background,track_surface,line");
    const auto count = root["num_classes"];
    if (!count.isInt() || static_cast<int>(count) != static_cast<int>(known.size())) bad("num_classes sai");
    const auto values = root["mask_values"];
    if (!values.isMap()) bad("thieu mask_values");
    for (size_t i = 0; i < known.size(); ++i) {
        const auto value = values[known[i]];
        if (!value.isInt() || static_cast<int>(value) != static_cast<int>(i)) bad("mask_values lech thu tu lop");
    }

    pair(root, "source_image_hw", info.sourceH, info.sourceW);
    pair(root, "crop_y", info.cropY0, info.cropY1);
    pair(root, "crop_x", info.cropX0, info.cropX1);
    pair(root, "input_hw", info.inputH, info.inputW);
    if (info.sourceH <= 0 || info.sourceW <= 0) bad("source_image_hw vo ly");
    if (info.cropY0 < 0 || info.cropY1 <= info.cropY0 || info.cropY1 > info.sourceH) bad("crop_y vo ly");
    if (info.cropX0 < 0 || info.cropX1 <= info.cropX0 || info.cropX1 > info.sourceW) bad("crop_x vo ly");
    if (info.inputH < 8 || info.inputW < 8 || info.inputH > 1024 || info.inputW > 1024) bad("input_hw vo ly");
    const int input[4] = {1, 3, info.inputH, info.inputW};
    const int output[4] = {1, static_cast<int>(known.size()), info.inputH, info.inputW};
    shape(root, "input_shape", input);
    shape(root, "output_shape", output);
    const auto normalization = root["normalization"];
    if (!normalization.isMap() || !normalization["scale"].isReal() ||
        std::abs(static_cast<double>(normalization["scale"]) - 1.0 / 255.0) > 1e-6)
        bad("normalization.scale phai la 1/255");
    return info;
}

std::string defaultInfoPath(const std::string& onnxPath) {
    if (endsWith(onnxPath, kCvdnnSuffix))
        return onnxPath.substr(0, onnxPath.size() - std::string(kCvdnnSuffix).size()) + "_info.json";
    if (endsWith(onnxPath, kOnnxSuffix))
        return onnxPath.substr(0, onnxPath.size() - std::string(kOnnxSuffix).size()) + "_info.json";
    return onnxPath + "_info.json";
}

bool isCvdnnFile(const std::string& onnxPath) { return endsWith(onnxPath, kCvdnnSuffix); }
} // namespace model_view
