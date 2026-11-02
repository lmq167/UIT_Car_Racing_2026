#pragma once
#include "ModelInfo.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace model_view {
constexpr int kMaskZoom = 4;
constexpr int kThumbWidth = 640;
constexpr int kThumbHeight = 360;
constexpr double kOverlayAlpha = 0.5;
// Dòng chữ đầu của panel: hai dòng trên cùng do PreviewHost vẽ trạng thái xe.
constexpr int kPanelTextY = 70;

// BGR: background (0,0,0), track_surface (0,160,0), line (0,255,255).
cv::Mat colorizeMask(const cv::Mat& mask, const std::vector<std::string>& classNames);
cv::Mat renderMaskPanel(const cv::Mat& mask, const ModelInfo& info);
// Phủ mặt nạ lên đúng vùng crop của thumb và vẽ vạch đỏ ở crop_y đầu.
cv::Mat renderOverlayPanel(const cv::Mat& thumb, const cv::Mat& mask, const ModelInfo& info);
cv::Mat renderNotice(const std::string& reason);
// Font Hershey chỉ vẽ ASCII: bỏ dấu không vẽ được thay vì hiện dấu hỏi.
std::string asciiOnly(const std::string& text);
void drawLine(cv::Mat& panel, int line, const std::string& text);
} // namespace model_view
