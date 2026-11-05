#pragma once
// Chép dữ liệu điều khiển sang PreviewDebug cho --debug-windows; thuần, chỉ đọc đầu vào.
#include "../collect_common/preview.h"
#include "control/LaneController.h"
#include "control/Detections.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>

namespace auto_drive {
inline void fillPreviewBoxes(drive::PreviewDebug& out, const control::Detections& det,
                             int64_t now_ms, int64_t max_age_ms, bool yolo_enabled) {
    out.yolo_state = yolo_enabled ? 1 : 0;
    out.det_ms = det.captured_ms;
    out.nboxes = 0;
    if (!yolo_enabled || det.captured_ms < 0 || now_ms - det.captured_ms > max_age_ms) return;
    out.nboxes = int(std::min<size_t>(det.items.size(), drive::kDebugMaxBoxes));
    for (int i = 0; i < out.nboxes; ++i) {
        const auto& in = det.items[i];
        auto& box = out.boxes[i];
        box.x = int32_t(std::lround(in.box.x)); box.y = int32_t(std::lround(in.box.y));
        box.w = int32_t(std::lround(in.box.width)); box.h = int32_t(std::lround(in.box.height));
        box.confidence = in.confidence;
        std::snprintf(box.label, sizeof(box.label), "%.15s", in.class_name.c_str());
    }
}

// plan có thể null (chưa có hàng quét). Mask rỗng/quá sức chứa thì mask_w = mask_h = 0.
inline void fillPreviewDebug(drive::PreviewDebug& out, const cv::Mat& mask, int64_t mask_ms, uint64_t frame_id,
                             const control::ControlDebug* plan, double center_offset_frac,
                             cv::Size source, cv::Rect crop) {
    out.mask_ms = mask_ms; out.frame_id = frame_id;
    out.src_w = source.width; out.src_h = source.height;
    out.crop_x = crop.x; out.crop_y = crop.y; out.crop_w = crop.width; out.crop_h = crop.height;
    const bool usable = !mask.empty() && mask.type() == CV_8UC1 && mask.isContinuous() &&
                        mask.total() <= size_t(drive::kDebugMaskBytes);
    out.mask_w = usable ? mask.cols : 0; out.mask_h = usable ? mask.rows : 0;
    if (usable) std::memcpy(out.mask, mask.data, mask.total());
    out.ref_x = float(mask.cols / 2.0 + center_offset_frac * mask.cols);
    out.nrows = 0; out.valid_rows = 0; out.nboxes = 0;
    out.e = out.p = out.d = out.angle_deg = out.speed_cms = 0;
    std::memset(out.state, 0, sizeof(out.state)); std::memset(out.reason, 0, sizeof(out.reason));
    if (!plan) return;
    out.valid_rows = plan->valid_rows;
    out.e = float(plan->e); out.p = float(plan->p); out.d = float(plan->d);
    out.angle_deg = float(plan->angle_deg); out.speed_cms = float(plan->speed_cms);
    std::snprintf(out.state, sizeof(out.state), "%s", plan->state.c_str());
    std::snprintf(out.reason, sizeof(out.reason), "%s", plan->reason.c_str());
    out.nrows = int(std::min<size_t>(plan->rows.size(), drive::kDebugMaxRows));
    for (int i = 0; i < out.nrows; ++i) {
        const auto& in = plan->rows[i];
        auto& row = out.rows[i];
        row.y = in.y; row.left = float(in.left); row.right = float(in.right); row.center = float(in.center);
        row.valid = in.valid ? 1 : 0;
        row.nlines = int(std::min<size_t>(in.lines.size(), drive::kDebugMaxLines));
        for (int k = 0; k < row.nlines; ++k) row.lines[k] = float(in.lines[k]);
    }
}
}
