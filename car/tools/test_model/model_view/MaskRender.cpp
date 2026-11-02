#include "MaskRender.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace model_view {
namespace {
cv::Vec3b classColor(const std::string& name) {
    if (name == "background") return {0, 0, 0};
    if (name == "track_surface") return {0, 160, 0};
    if (name == "line") return {0, 255, 255};
    throw std::runtime_error("Lop khong co mau: " + name);
}
} // namespace

cv::Mat colorizeMask(const cv::Mat& mask, const std::vector<std::string>& classNames) {
    if (mask.type() != CV_8UC1) throw std::runtime_error("Mat na phai la CV_8UC1");
    std::vector<cv::Vec3b> palette;
    for (const auto& name : classNames) palette.push_back(classColor(name));
    cv::Mat color(mask.size(), CV_8UC3);
    for (int y = 0; y < mask.rows; ++y) {
        const auto* in = mask.ptr<unsigned char>(y);
        auto* out = color.ptr<cv::Vec3b>(y);
        for (int x = 0; x < mask.cols; ++x) {
            if (in[x] >= palette.size()) throw std::runtime_error("Chi so lop vuot danh sach lop");
            out[x] = palette[in[x]];
        }
    }
    return color;
}

cv::Mat renderMaskPanel(const cv::Mat& mask, const ModelInfo& info) {
    cv::Mat panel;
    cv::resize(colorizeMask(mask, info.classNames), panel, cv::Size(), kMaskZoom, kMaskZoom, cv::INTER_NEAREST);
    return panel;
}

cv::Mat renderOverlayPanel(const cv::Mat& thumb, const cv::Mat& mask, const ModelInfo& info) {
    cv::Mat panel = thumb.clone();
    const double sy = double(thumb.rows) / info.sourceH, sx = double(thumb.cols) / info.sourceW;
    const int y0 = int(std::lround(info.cropY0 * sy)), y1 = int(std::lround(info.cropY1 * sy));
    const int x0 = int(std::lround(info.cropX0 * sx)), x1 = int(std::lround(info.cropX1 * sx));
    const cv::Rect region(x0, y0, x1 - x0, y1 - y0);
    cv::Mat color;
    cv::resize(colorizeMask(mask, info.classNames), color, region.size(), 0, 0, cv::INTER_NEAREST);
    cv::Mat target = panel(region);
    cv::addWeighted(color, kOverlayAlpha, target, 1.0 - kOverlayAlpha, 0, target);
    cv::line(panel, cv::Point(0, y0), cv::Point(panel.cols - 1, y0), cv::Scalar(0, 0, 255), 1);
    return panel;
}

std::string asciiOnly(const std::string& text) {
    std::string out;
    for (unsigned char c : text) if (c >= 32 && c < 127) out += char(c);
    return out;
}

void drawLine(cv::Mat& panel, int line, const std::string& text) {
    cv::putText(panel, asciiOnly(text), cv::Point(8, kPanelTextY + line * 22), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1);
}

cv::Mat renderNotice(const std::string& reason) {
    cv::Mat panel(kThumbHeight, kThumbWidth, CV_8UC3, cv::Scalar(40, 40, 40));
    drawLine(panel, 0, "KHONG CO MODEL:");
    const auto text = asciiOnly(reason);
    const size_t width = 70;
    for (size_t i = 0, line = 1; i < text.size() && line < 12; i += width, ++line)
        drawLine(panel, int(line), text.substr(i, width));
    return panel;
}
} // namespace model_view
