#pragma once
// Hàm vẽ thuần cho cửa sổ gỡ lỗi của ucr_auto (--debug-windows): đầu vào là khung + PreviewDebug,
// đầu ra là ảnh; không đụng bộ nhớ chung, tín hiệu hay phím. Chỉ dùng chữ ASCII (putText không vẽ dấu).
#include "preview.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace drive {

inline bool debugMaskUsable(const PreviewDebug& d) {
    return d.mask_w > 0 && d.mask_h > 0 && int64_t(d.mask_w) * d.mask_h <= kDebugMaskBytes;
}
// Lớp 0 nền xám, 1 và 2 đều là mặt đường xanh dương (BGR); khác tông với dấu xanh lá/đỏ của hàng hành lang.
inline cv::Scalar debugClassColor(int value) {
    return value >= 1 ? cv::Scalar(200, 100, 30) : cv::Scalar(40, 40, 40);
}
inline cv::Mat colorizeDebugMask(const PreviewDebug& d) {
    cv::Mat color(d.mask_h, d.mask_w, CV_8UC3);
    for (int y = 0; y < d.mask_h; ++y)
        for (int x = 0; x < d.mask_w; ++x) {
            const cv::Scalar c = debugClassColor(d.mask[size_t(y) * d.mask_w + x]);
            color.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(c[0]), uchar(c[1]), uchar(c[2]));
        }
    return color;
}
inline void debugText(cv::Mat& image, const std::string& text, cv::Point at, cv::Scalar color) {
    cv::putText(image, text, at + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
}

// Cửa sổ 1: mask U-Net tô màu, phóng `scale` lần (mask 160x96 -> 320x192).
inline cv::Mat renderDebugMask(const PreviewDebug& d, int scale = 2) {
    scale = std::max(1, std::min(scale, 8));
    if (!debugMaskUsable(d)) {
        cv::Mat blank(96 * scale, 160 * scale, CV_8UC3, debugClassColor(0));
        debugText(blank, "CHUA CO MASK", cv::Point(8, 20), cv::Scalar(255, 255, 255));
        return blank;
    }
    cv::Mat big;
    cv::resize(colorizeDebugMask(d), big, cv::Size(d.mask_w * scale, d.mask_h * scale), 0, 0, cv::INTER_NEAREST);
    debugText(big, "0 nen  1/2 mat duong", cv::Point(4, big.rows - 6), cv::Scalar(255, 255, 255));
    return big;
}

// Truy vết (P5), hàm thuần: các dòng chữ ASCII + cờ cảnh báo (vẽ đỏ). frame_ms = giờ chụp của khung đang hiển thị.
// Khung hiển thị là khung camera MỚI NHẤT; mask/hành lang/trạng thái thuộc khung bộ điều khiển đã dùng (mask_ms):
// dòng "hien thi moi hon" cho biết hai khung lệch nhau bao nhiêu ms.
// Ngưỡng đổi màu (log m4: lệch hiển thị p95 ~130 ms, tuổi khung quyết định p50 ~150, p95 ~210 ms).
constexpr int64_t kDebugLagWarnMs = 150, kDebugDecisionWarnMs = 250, kDebugSensorWarnMs = 200;
struct DebugInfoLine { std::string text; bool warn = false; };
inline std::vector<DebugInfoLine> debugInfoLines(const PreviewDebug& d, int64_t frame_ms) {
    std::vector<DebugInfoLine> out;
    char b[128];
    if (d.log_t0_ms >= 0 && d.decision_ms >= d.log_t0_ms)
        std::snprintf(b, sizeof(b), "s=%.1f #%llu %s", (d.decision_ms - d.log_t0_ms) / 1000.0,
                      (unsigned long long)d.frame_id, d.recording ? "DANG GHI" : "khong ghi anh");
    else std::snprintf(b, sizeof(b), "s=-- #%llu %s", (unsigned long long)d.frame_id, d.recording ? "DANG GHI" : "khong ghi anh");
    out.push_back({b, false});
    const long long decided = d.decision_ms >= 0 && d.mask_ms >= 0 ? (long long)(d.decision_ms - d.mask_ms) : -1;
    const long long lag = d.mask_ms >= 0 && frame_ms >= d.mask_ms ? (long long)(frame_ms - d.mask_ms) : -1;
    std::snprintf(b, sizeof(b), "khung QD tuoi=%lldms | hien thi moi hon %lldms", decided, lag);
    out.push_back({b, lag > kDebugLagWarnMs || decided > kDebugDecisionWarnMs});
    const bool imu_bad = !d.sensor_ok || d.sensor_age_ms < 0 || d.sensor_age_ms > kDebugSensorWarnMs;
    if (d.sensor_ok) std::snprintf(b, sizeof(b), "IMU yaw=%.1f odo=%.1fcm tuoi=%dms", double(d.yaw_deg), double(d.odo_cm), int(d.sensor_age_ms));
    else std::snprintf(b, sizeof(b), "IMU/encoder LOI (tuoi=%dms)", int(d.sensor_age_ms));
    out.push_back({b, imu_bad});
    char note[sizeof(d.route_note) + 1] = {}, end[sizeof(d.turn_end) + 1] = {};
    std::memcpy(note, d.route_note, sizeof(d.route_note)); std::memcpy(end, d.turn_end, sizeof(d.turn_end));
    std::snprintf(b, sizeof(b), "lo trinh k=%d y_dinh=%s %s", int(d.route_step),
                  d.turn_dir > 0 ? "TRAI" : d.turn_dir < 0 ? "PHAI" : "-", note);
    out.push_back({b, false});
    if (d.turn_active) {
        const double span = double(d.turn_yaw_tgt) - d.turn_yaw0;
        const int pct = std::abs(span) > 1e-3 ? int(std::lround(100.0 * (1.0 - double(d.turn_yaw_err) / span))) : 0;
        std::snprintf(b, sizeof(b), "RE yaw0=%.0f dich=%.0f con=%.0f tien do=%d%%", double(d.turn_yaw0),
                      double(d.turn_yaw_tgt), double(d.turn_yaw_err), pct);
    } else std::snprintf(b, sizeof(b), "RE: - %s", end);
    out.push_back({b, false});
    return out;
}

// Cửa sổ 2: phủ mask (alpha), hành lang (biên trái/phải, target), lookahead, đích, lệnh, trạng thái và khung YOLO.
inline void drawDebugOverlay(cv::Mat& display, const PreviewDebug& d, const PreviewState& s, int64_t frame_ms) {
    if (display.empty() || display.type() != CV_8UC3) return;
    const bool mask_ok = debugMaskUsable(d);
    const int src_w = d.src_w > 0 ? d.src_w : (mask_ok ? d.mask_w : display.cols);
    const int src_h = d.src_h > 0 ? d.src_h : (mask_ok ? d.mask_h : display.rows);
    const bool crop_ok = d.crop_w > 0 && d.crop_h > 0;
    const double cx = crop_ok ? d.crop_x : 0, cy = crop_ok ? d.crop_y : 0;
    const double cw = crop_ok ? d.crop_w : src_w, ch = crop_ok ? d.crop_h : src_h;
    const double sx = double(display.cols) / src_w, sy = double(display.rows) / src_h;
    const double mw = mask_ok ? d.mask_w : 1, mh = mask_ok ? d.mask_h : 1;
    // Toạ độ mask -> điểm ảnh trên khung hiển thị.
    const auto X = [&](double mx) { return int(std::lround((cx + mx * cw / mw) * sx)); };
    const auto Y = [&](double my) { return int(std::lround((cy + (my + 0.5) * ch / mh) * sy)); };
    const cv::Rect region(int(std::lround(cx * sx)), int(std::lround(cy * sy)),
                          int(std::lround(cw * sx)), int(std::lround(ch * sy)));
    const cv::Rect bounds(0, 0, display.cols, display.rows);
    const cv::Rect visible = region & bounds;
    if (mask_ok && !visible.empty()) {
        cv::Mat layer;
        cv::resize(colorizeDebugMask(d), layer, region.size(), 0, 0, cv::INTER_NEAREST);
        cv::Mat classes(d.mask_h, d.mask_w, CV_8UC1, const_cast<uint8_t*>(d.mask)), covered, big;
        cv::compare(classes, 0, covered, cv::CMP_GT); // Nền (lớp 0) để trong suốt.
        cv::resize(covered, big, region.size(), 0, 0, cv::INTER_NEAREST);
        const cv::Rect inner(visible.x - region.x, visible.y - region.y, visible.width, visible.height);
        cv::Mat target = display(visible), blended;
        cv::addWeighted(target, 0.6, layer(inner), 0.4, 0, blended);
        blended.copyTo(target, big(inner));
    }
    const int rows = std::max(0, std::min<int>(d.nrows, kDebugMaxRows));
    for (int i = 0; i < rows; ++i) {
        const auto& row = d.rows[i];
        const cv::Scalar color = row.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        const int y = Y(row.y);
        cv::line(display, cv::Point(region.x, y), cv::Point(region.x + region.width, y), color, 1);
        for (double edge : {double(row.left), double(row.right)})
            if (edge >= 0) cv::line(display, cv::Point(X(edge), y - 4), cv::Point(X(edge), y + 4), cv::Scalar(255, 128, 0), 2);
        if (row.target >= 0) cv::circle(display, cv::Point(X(row.target), y), 4, color, row.valid ? -1 : 2);
    }
    if (mask_ok) {
        // Trục tham chiếu của xe (trắng) và đích điều khiển = trục + sai số e (tím).
        cv::line(display, cv::Point(X(d.ref_x), region.y), cv::Point(X(d.ref_x), region.y + region.height),
                 cv::Scalar(255, 255, 255), 1);
        if (d.look_x >= 0) // Điểm lookahead (trắng, vòng).
            cv::circle(display, cv::Point(X(d.look_x), Y(d.look_y)), 7, cv::Scalar(255, 255, 255), 2);
        if (d.valid_rows > 0)
            cv::circle(display, cv::Point(X(d.ref_x + d.e * mw / 2), region.y + region.height - 10), 6,
                       cv::Scalar(255, 0, 255), -1);
    }
    const int boxes = std::max(0, std::min<int>(d.nboxes, kDebugMaxBoxes));
    for (int i = 0; i < boxes; ++i) {
        const auto& box = d.boxes[i];
        const cv::Rect r(int(std::lround(box.x * sx)), int(std::lround(box.y * sy)),
                         int(std::lround(box.w * sx)), int(std::lround(box.h * sy)));
        cv::rectangle(display, r, cv::Scalar(0, 165, 255), 2);
        char label[40];
        std::snprintf(label, sizeof(label), "%.15s %.2f", box.label, double(box.confidence));
        debugText(display, label, cv::Point(r.x, std::max(12, r.y - 4)), cv::Scalar(0, 165, 255));
    }
    char state[sizeof(d.state) + 1] = {}, reason[sizeof(d.reason) + 1] = {};
    std::memcpy(state, d.state, sizeof(d.state));
    std::memcpy(reason, d.reason, sizeof(d.reason));
    char line[128];
    const int base = display.rows - 8;
    if (d.yolo_state == 0) std::snprintf(line, sizeof(line), "YOLO: TAT");
    else if (d.det_ms < 0) std::snprintf(line, sizeof(line), "YOLO: OK tuoi=-- hop=%d", boxes);
    else std::snprintf(line, sizeof(line), "YOLO: OK tuoi=%lldms hop=%d", (long long)(frame_ms - d.det_ms), boxes);
    debugText(display, line, cv::Point(8, base - 54), cv::Scalar(255, 255, 255));
    std::snprintf(line, sizeof(line), "%s | %s", state, reason);
    debugText(display, line, cv::Point(8, base - 36), cv::Scalar(255, 255, 255));
    std::snprintf(line, sizeof(line), "gui: v=%.1f cm/s  lai=%.0f do", s.cmd_speed, s.cmd_steer);
    debugText(display, line, cv::Point(8, base - 18), cv::Scalar(0, 255, 255));
    std::snprintf(line, sizeof(line), "hang %d/%d e=%.2f p=%.1f d=%.1f mask_tuoi=%lldms", d.valid_rows, d.nrows,
                  double(d.e), double(d.p), double(d.d), (long long)(d.mask_ms >= 0 ? frame_ms - d.mask_ms : -1));
    debugText(display, line, cv::Point(8, base), cv::Scalar(0, 255, 0));
    // Góc trên phải: truy vết (không che vùng làn ở nửa dưới).
    const auto info = debugInfoLines(d, frame_ms);
    const int x0 = std::max(8, display.cols - 400);
    for (size_t i = 0; i < info.size(); ++i)
        debugText(display, info[i].text, cv::Point(x0, 20 + int(i) * 18),
                  info[i].warn ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0));
}

} // namespace drive
