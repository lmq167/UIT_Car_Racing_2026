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
    int kd_filter_ms;
    int obstacle_action, work_barrier_action;
    double work_barrier_y2_frac;
    double obstacle_hole_frac, obstacle_pass_cm;
    int start_wait_green, vote_window, vote_min, green_frames;
    double red_conf_min, green_conf_min, off_conf_min;
    double red_box_min_area_frac, green_box_min_area_frac, off_box_min_area_frac;
    double approach_cms, red_stop_x_frac;
    int turn_enable, turn_max_ms;
    double turn_yaw_deg, turn_steer_deg, turn_arm_cm, turn_max_cm, turn_start_cm;
    double rb_lost_cm, rb_turn1_cm, rb_turn2_cm, rb_yaw_kp, rb_hold_max_cm;

    static ControlParams Load(const std::string &path, std::ostream &report);
    void Validate() const;
    void PrintTable(std::ostream &out) const;
    static std::string FileHash(const std::string &path);
};
}
