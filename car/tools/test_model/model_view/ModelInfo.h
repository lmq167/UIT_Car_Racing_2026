#pragma once
#include <string>
#include <vector>

namespace model_view {
// Trường theo ô xuất ONNX của road_train_car.ipynb bản master.
struct ModelInfo {
    std::string name;
    std::vector<std::string> classNames;
    int sourceH = 0, sourceW = 0;
    int cropY0 = 0, cropY1 = 0, cropX0 = 0, cropX1 = 0;
    int inputH = 0, inputW = 0;
    bool imagenetInModel = false, fakeData = false;
};
// Ném std::runtime_error kèm lý do khi thiếu trường, cỡ vô lý hoặc lớp lạ.
ModelInfo loadModelInfo(const std::string& path);
// Mặc định: cạnh .onnx, cùng tên + _info.json; bản <tên>_cvdnn.onnx dùng info của <tên>.
std::string defaultInfoPath(const std::string& onnxPath);
// shortcut: nhận biết bản chuyển đổi theo tên tệp, đổi tên tệp thì mất cảnh báo; đọc opset nếu cần chắc hơn.
bool isCvdnnFile(const std::string& onnxPath);
} // namespace model_view
