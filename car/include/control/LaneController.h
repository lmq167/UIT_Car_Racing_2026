#pragma once
#include "control/ControlParams.h"
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace control {
// Chế độ quỹ đạo do Planner chọn; LaneController chỉ đổi điểm mục tiêu theo chế độ.
enum class TrajMode { LANE, AVOID_LEFT, RETURN_RIGHT, ROUNDABOUT, TURN_LEFT, TURN_RIGHT };
const char *TrajModeName(TrajMode mode);  // "BAM_LAN","NE_TRAI","VE_PHAI","VONG_XUYEN"
struct Command { double cms=0, deg=0; };
// Một hàng hành lang (pixel mask). flag_*: "THAT" | "MEP_ANH" | "NHAY".
struct RowDebug {
    int y=0;
    double left=-1, right=-1, target=-1, conf=0;
    double raw_l=-1, raw_r=-1;    // đoạn thô được chọn (trước khi thay biên không THAT)
    double hole_l=-1, hole_r=-1;  // khe lớn nhất khi hole (pixel 0 đầu/cuối của khe)
    std::string flag_left, flag_right;
    int lanes=0;
    bool hole=false, avoid_ok=true, valid=false;
};
struct Corridor { std::vector<RowDebug> rows; };
struct ControlDebug {
    std::vector<RowDebug> rows;
    double look_x=-1, look_y=-1;   // pixel, sau kẹp hành lang và giới hạn nhảy; look_x<0: khung không hợp lệ
    std::vector<double> fit;       // x = fit[0] + fit[1]·y + fit[2]·y² (y pixel); rỗng khi không hợp lệ
    std::string mode="BAM_LAN";
    double e=0, p=0, d=0, angle_deg=0, speed_cms=0;
    int valid_rows=0;
    std::string state="MAT_LAN", reason="CHUA_CO_LAN";
    int route_step=0, turn_dir=0;  // Planner điền sau Step, chỉ để ghi log chẩn đoán
    std::string route_note;
};
struct ControlResult { Command cmd; ControlDebug debug; };
// Hàm thuần, không trạng thái: hành lang theo hàng từ mask 0/1 (2 cũng là đường).
// prior_lanes: số làn đã thấy rõ ở khung trước (0 chưa biết, 1, 2); chỉ dùng khi biên phải chạm mép ảnh ở chế độ LANE.
Corridor ExtractCorridor(const cv::Mat &mask, const ControlParams &params, TrajMode mode, int prior_lanes=0);
class LaneController {
public:
    ControlResult Step(const cv::Mat &mask, int64_t mask_ms, int64_t now_ms,
                       const ControlParams &params, TrajMode mode=TrajMode::LANE, bool wide_center=true);
    void Reset();
private:
    int64_t last_now_ms_=-1, last_mask_ms_=-1, last_valid_ms_=-1, regain_mask_ms_=-1;
    double angle_=0, error_=0;
    bool lost_=false, derivative_ready_=false;
    int regain_=0;
    TrajMode mode_=TrajMode::LANE;
    double look_x_=0;            // lookahead khung hợp lệ trước (giới hạn nhảy)
    bool have_look_=false;
    int lanes_memory_=0;         // số làn thấy rõ (cả hai biên thật) ở khung hợp lệ gần nhất; Reset xóa
    TrajMode look_mode_=TrajMode::LANE;
};
}
