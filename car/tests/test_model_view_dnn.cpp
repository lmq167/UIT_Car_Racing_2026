// Suy luận DNN thật trên ONNX giả (trọng số ngẫu nhiên): chỉ kiểm đường chạy, KHÔNG kiểm chất lượng model.
#include "DnnMaskModel.h"
#include <opencv2/core/utility.hpp>
#include <fstream>
#include <iostream>
#include <string>

#ifdef UCR_TSAN_SUPP
// Như harness T33: race giả trong OpenCV/TBB chưa instrument, xem tools/drive_collect/tsan.supp.
extern "C" const char* __tsan_default_options() { return "suppressions=" UCR_TSAN_SUPP; }
#endif

int main(int argc, char** argv) {
    if (argc != 4) { std::cerr << "Dung: test_model_view_dnn <_cvdnn.onnx> <_info.json> <resize12.onnx>\n"; return 2; }
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        if (!ok) { std::cerr << "HONG: " << what << '\n'; ++failures; }
    };
    const auto info = model_view::loadModelInfo(argv[2]);
    model_view::DnnMaskModel model(argv[1], info);
    cv::Mat frame(720, 1280, CV_8UC3);
    cv::randu(frame, 0, 255);
    const cv::Mat mask = model.infer(frame);
    check(mask.type() == CV_8UC1 && mask.rows == info.inputH && mask.cols == info.inputW, "mask dung input_hw");
    double low = 0, high = 0;
    cv::minMaxLoc(mask, &low, &high);
    check(high < 3, "chi so lop < 3");
    std::cout << "OpenCV " << CV_VERSION << ": _cvdnn chay duoc, lop " << low << ".." << high << '\n';
    // Bản gốc opset 12: OpenCV 4.1.1 không đọc được; bản mới hơn có thể đọc. Chỉ yêu cầu báo lỗi có kiểm soát.
    try {
        model_view::DnnMaskModel original(argv[3], info);
        original.infer(frame);
        std::cout << "ONNX goc opset 12: doc duoc tren ban OpenCV nay\n";
    } catch (const cv::Exception& e) {
        std::cout << "ONNX goc opset 12: loi co kiem soat (" << e.err.substr(0, 80) << ")\n";
    }
    const std::string broken = std::string(argv[1]) + ".broken.onnx";
    std::ofstream(broken) << "khong phai onnx";
    try { model_view::DnnMaskModel bad(broken, info); check(false, "ONNX hong phai nem"); }
    catch (const cv::Exception&) {}
    catch (const std::runtime_error&) {}
    if (failures) return 1;
    std::cout << "OK model_view dnn\n";
    return 0;
}
