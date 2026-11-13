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
    // Hành lang (T79): hàng quét, bề rộng một làn kỳ vọng [gần, xa], ngưỡng hai làn/nhảy/tin cậy, lookahead.
    int corridor_rows;
    int lane_backup;
    double lane_backup_right_frac; // TẠM, CHƯA ĐO
    double corridor_bottom_frac, corridor_top_frac;
    std::vector<double> lane1_width_frac;
    double two_lane_ratio, max_jump_frac, conf_min, lookahead_frac, avoid_min_width_ratio;
    double center_offset_frac;
    int min_valid_rows, lost_hold_ms, lost_stop_ms, regain_frames;
    int record_every_n, yolo_every_n;
    int obstacle_action, work_barrier_action;
    double work_barrier_y2_frac;
    double obstacle_hole_ratio, obstacle_pass_cm, obstacle_near_w_frac;
    int start_wait_green, vote_window, vote_min, green_frames;
    double red_conf_min, green_conf_min, off_conf_min;
    double red_box_min_area_frac, green_box_min_area_frac, off_box_min_area_frac;
    double approach_cms, red_stop_x_frac;
    int turn_enable, turn_max_ms, sign_seq_next;
    int imu_fallback;
    double turn_yaw_deg, turn_steer_deg, turn_arm_cm, turn_max_cm, turn_start_cm, turn_left_delay_cm, route_t2_gap_cm, st7_left_bias_deg;
    double rb_lost_cm, rb_turn1_cm, rb_turn2_cm, rb_yaw_kp, rb_hold_max_cm;
    // Đỗ xe lùi song song (chặng 8, Quân duyệt viết với số tạm 11/11): mặc định tắt.
    int park_enable, park_side, park_tof_map;
    double park_forward_cm, park_arc_deg, park_rev_cms, park_steer_deg, park_entry_cm, park_cell_cm;

    static ControlParams Load(const std::string &path, std::ostream &report);
    void Validate() const;
    void PrintTable(std::ostream &out) const;
    static std::string FileHash(const std::string &path);
};
}
