#pragma once
#include <iosfwd>
#include <string>
#include <vector>

namespace control {
struct ControlParams {
    ControlParams();
    int control_period_ms, mask_max_age_ms, cmd_max_age_ms, det_max_age_ms;
    double max_speed_cms, cruise_cms, min_cms, lost_cms, slow_per_deg;
    double steer_max_deg, steer_rate_dps, kp, kd;
    std::vector<double> scan_rows, row_weights, lane_width_frac;
    double lane_width_min_ratio, lane_width_max_ratio, center_offset_frac;
    int min_valid_rows, lost_hold_ms, lost_stop_ms, regain_frames;
    int record_every_n, yolo_every_n;
    double right_clip_frac;
    int right_lost_ms;
    int pair_mode;
    double pair_max_shift_frac, pair_merge_frac;

    static ControlParams Load(const std::string &path, std::ostream &report);
    void Validate() const;
    void PrintTable(std::ostream &out) const;
    static std::string FileHash(const std::string &path);
};
}
