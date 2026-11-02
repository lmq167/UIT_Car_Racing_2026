// Hàm thuần của model_view: đọc _info.json, tiền xử lý, argmax, tô màu, phủ ảnh. Giá trị kỳ vọng tính tay.
#include "DnnMaskModel.h"
#include "MaskRender.h"
#include "ModelInfo.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
int failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) { std::cerr << "HONG: " << what << '\n'; ++failures; }
}
template <class F> void throws(F f, const std::string& needle, const std::string& what) {
    try { f(); check(false, what + " (khong nem)"); }
    catch (const std::runtime_error& e) {
        check(std::string(e.what()).find(needle) != std::string::npos, what + " (ly do: " + e.what() + ")");
    }
}
const char* kInfo = R"J({"task":"segmentation","architecture":"unet_b_minus","class_names":["background","track_surface","line"],
"num_classes":3,"mask_values":{"background":0,"track_surface":1,"line":2},"source_image_hw":[720,1280],
"crop_y":[320,720],"crop_x":[0,1280],"resize":"cv2.resize INTER_AREA (crop -> input)","input_name":"images",
"input_shape":[1,3,96,160],"input_hw":[96,160],"input_dtype":"float32","color_order":"RGB","layout":"NCHW",
"normalization":{"scale":0.00392156862745098,"mean":[0,0,0],"std":[1,1,1]},"imagenet_normalization_in_model":false,
"preprocessing_in_model":false,"output_name":"logits","output_shape":[1,3,96,160],"output_type":"logits",
"activation":"softmax_over_channels_argmax","opset":12,"dynamic":false,"fake_data":true,"variant":"S1"})J";

std::string writeInfo(const std::string& name, std::string text, const std::string& from = "", const std::string& to = "") {
    if (!from.empty()) {
        const auto at = text.find(from);
        if (at == std::string::npos) throw std::logic_error("mau thay the khong co: " + from);
        text.replace(at, from.size(), to);
    }
    const std::string path = "t42_core_" + name + "_info.json";
    std::ofstream(path) << text;
    return path;
}

void infoCases() {
    const auto info = model_view::loadModelInfo(writeInfo("ok", kInfo));
    check(info.inputH == 96 && info.inputW == 160, "input_hw");
    check(info.cropY0 == 320 && info.cropY1 == 720 && info.cropX0 == 0 && info.cropX1 == 1280, "crop");
    check(info.sourceH == 720 && info.sourceW == 1280, "source_image_hw");
    check(info.classNames.size() == 3 && info.classNames[2] == "line", "class_names");
    check(info.fakeData && !info.imagenetInModel, "co true/false");
    check(info.name == "S1/unet_b_minus", "ten model");
    const auto s4 = model_view::loadModelInfo(writeInfo("s4", kInfo, "\"imagenet_normalization_in_model\":false",
                                                         "\"imagenet_normalization_in_model\":true"));
    check(s4.imagenetInModel, "doc imagenet_normalization_in_model");
    throws([] { model_view::loadModelInfo("khong_co_file_info.json"); }, "_info.json", "thieu file");
    throws([] { model_view::loadModelInfo(writeInfo("broken", "{\"task\": [")); }, "_info.json", "JSON hong");
    throws([] { model_view::loadModelInfo(writeInfo("nocrop", kInfo, "\"crop_y\":[320,720],", "")); },
           "crop_y", "thieu crop_y");
    throws([] { model_view::loadModelInfo(writeInfo("class", kInfo, "\"line\"]", "\"lane\"]")); },
           "lop la", "lop la");
    throws([] { model_view::loadModelInfo(writeInfo("crop", kInfo, "[320,720]", "[320,900]")); },
           "crop_y", "crop vuot anh");
    throws([] { model_view::loadModelInfo(writeInfo("hw", kInfo, "\"input_hw\":[96,160]", "\"input_hw\":[0,160]")); },
           "input_hw", "input_hw vo ly");
    throws([] { model_view::loadModelInfo(writeInfo("shape", kInfo, "\"output_shape\":[1,3,96,160]",
                                                    "\"output_shape\":[1,1,96,160]")); },
           "output_shape", "output_shape sai so lop");
    throws([] { model_view::loadModelInfo(writeInfo("act", kInfo, "softmax_over_channels_argmax", "sigmoid_per_class")); },
           "activation", "activation cu");
    throws([] { model_view::loadModelInfo(writeInfo("scale", kInfo, "0.00392156862745098", "1.0")); },
           "scale", "chuan hoa sai");
    check(model_view::defaultInfoPath("/m/road_S1_s0.onnx") == "/m/road_S1_s0_info.json", "info mac dinh");
    check(model_view::defaultInfoPath("/m/road_S1_s0_cvdnn.onnx") == "/m/road_S1_s0_info.json", "info cua _cvdnn");
    check(model_view::isCvdnnFile("a_cvdnn.onnx") && !model_view::isCvdnnFile("a.onnx"), "nhan biet _cvdnn");
}

model_view::ModelInfo fakeInfo() {
    model_view::ModelInfo info;
    info.classNames = {"background", "track_surface", "line"};
    info.sourceH = 720; info.sourceW = 1280; info.cropY0 = 320; info.cropY1 = 720; info.cropX1 = 1280;
    info.inputH = 96; info.inputW = 160;
    return info;
}

void preprocessCases() {
    const auto info = fakeInfo();
    // Trên crop: trắng (không được lọt vào). Trong crop: nửa trái BGR(10,20,30), nửa phải BGR(200,100,50).
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
    frame(cv::Rect(0, 320, 640, 400)).setTo(cv::Scalar(10, 20, 30));
    frame(cv::Rect(640, 320, 640, 400)).setTo(cv::Scalar(200, 100, 50));
    const cv::Mat blob = model_view::makeInputBlob(frame, info);
    check(blob.dims == 4 && blob.size[0] == 1 && blob.size[1] == 3 && blob.size[2] == 96 && blob.size[3] == 160 &&
          blob.type() == CV_32F, "blob NCHW 1x3x96x160 float32");
    const auto at = [&](int c, int y, int x) { return blob.ptr<float>(0, c)[y * 160 + x]; };
    const auto near = [](float a, double b) { return std::abs(a - b) < 1e-5; };
    // Kênh 0 là R sau BGR→RGB.
    check(near(at(0, 0, 0), 30 / 255.0) && near(at(1, 0, 0), 20 / 255.0) && near(at(2, 0, 0), 10 / 255.0),
          "goc trai tren = RGB/255, khong lay hang tren crop_y");
    check(near(at(0, 95, 79), 30 / 255.0) && near(at(0, 95, 80), 50 / 255.0) && near(at(2, 50, 159), 200 / 255.0),
          "bien trai/phai dung cot sau INTER_AREA");
    throws([&] { model_view::makeInputBlob(cv::Mat(384, 640, CV_8UC3), info); }, "source_image_hw", "anh khac co");
    int dims[] = {1, 3, 96, 160};
    cv::Mat logits(4, dims, CV_32F, cv::Scalar(0));
    logits.ptr<float>(0, 1)[0] = 1.f;          // điểm 0 → lớp 1
    logits.ptr<float>(0, 2)[5] = 2.f;          // điểm 5 → lớp 2
    logits.ptr<float>(0, 0)[7] = -1.f;         // điểm 7: kênh 1 và 2 bằng nhau (0) → chọn kênh thấp hơn = 1
    const cv::Mat mask = model_view::argmaxMask(logits, info);
    check(mask.type() == CV_8UC1 && mask.rows == 96 && mask.cols == 160, "mask 96x160");
    check(mask.data[0] == 1 && mask.data[5] == 2 && mask.data[7] == 1 && mask.data[1] == 0, "argmax theo kenh");
    int wrong[] = {1, 1, 96, 160};
    throws([&] { model_view::argmaxMask(cv::Mat(4, wrong, CV_32F, cv::Scalar(0)), info); }, "output_shape",
           "dau ra 1 lop");
}

void renderCases() {
    const auto info = fakeInfo();
    cv::Mat mask(96, 160, CV_8UC1, cv::Scalar(1));
    mask.at<unsigned char>(0, 0) = 0;
    mask.at<unsigned char>(0, 1) = 2;
    const cv::Mat color = model_view::colorizeMask(mask, info.classNames);
    check(color.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 0), "mau background");
    check(color.at<cv::Vec3b>(5, 5) == cv::Vec3b(0, 160, 0), "mau track_surface");
    check(color.at<cv::Vec3b>(0, 1) == cv::Vec3b(0, 255, 255), "mau line");
    const cv::Mat panel = model_view::renderMaskPanel(mask, info);
    check(panel.rows == 384 && panel.cols == 640, "phong x4");
    check(panel.at<cv::Vec3b>(3, 3) == cv::Vec3b(0, 0, 0) && panel.at<cv::Vec3b>(3, 4) == cv::Vec3b(0, 255, 255) &&
          panel.at<cv::Vec3b>(3, 7) == cv::Vec3b(0, 255, 255) && panel.at<cv::Vec3b>(3, 8) == cv::Vec3b(0, 160, 0),
          "nearest: moi diem thanh khoi 4x4");
    cv::Mat bad = mask.clone();
    bad.at<unsigned char>(1, 1) = 3;
    throws([&] { model_view::colorizeMask(bad, info.classNames); }, "vuot", "chi so lop vuot");

    const cv::Mat thumb(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    const cv::Mat overlay = model_view::renderOverlayPanel(thumb, mask, info);
    // crop_y 320/720 → hàng 160 ở ảnh 640x360.
    check(overlay.at<cv::Vec3b>(159, 300) == cv::Vec3b(100, 100, 100), "tren crop_y giu nguyen");
    check(overlay.at<cv::Vec3b>(160, 300) == cv::Vec3b(0, 0, 255), "vach do o crop_y");
    check(overlay.at<cv::Vec3b>(161, 300) == cv::Vec3b(50, 130, 50), "phu alpha 0.5 track_surface");
    check(overlay.at<cv::Vec3b>(359, 639) == cv::Vec3b(50, 130, 50), "phu toi hang cuoi");
    check(overlay.at<cv::Vec3b>(162, 0) == cv::Vec3b(50, 50, 50), "diem (0,0) cua mask = background o goc trai");
    check(model_view::asciiOnly("KHÔNG có") == "KHNG c", "bo ky tu ngoai ASCII");
    check(!model_view::renderNotice("ly do").empty(), "panel thong bao");
}
} // namespace

int main() {
    try { infoCases(); preprocessCases(); renderCases(); }
    catch (const std::exception& e) { std::cerr << "HONG: ngoai le " << e.what() << '\n'; return 1; }
    if (failures) return 1;
    std::cout << "OK model_view core\n";
    return 0;
}
