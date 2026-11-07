add_executable(test_control_params test_control_params.cpp)
target_link_libraries(test_control_params PRIVATE ucr_control)
add_test(NAME control_params COMMAND test_control_params
    ${PROJECT_SOURCE_DIR}/control.yaml ${CMAKE_CURRENT_BINARY_DIR}/params_test.yaml)
add_executable(test_lane_controller test_lane_controller.cpp)
target_link_libraries(test_lane_controller PRIVATE ucr_control opencv_imgcodecs)
add_test(NAME control_lane COMMAND test_lane_controller)
add_executable(test_t65_clip_center test_t65_clip_center.cpp)
target_link_libraries(test_t65_clip_center PRIVATE ucr_control)
add_test(NAME acc_t65_clip_center COMMAND test_t65_clip_center)
set_tests_properties(acc_t65_clip_center PROPERTIES LABELS "control;acceptance" TIMEOUT 60)

# Một ca tạo run, gọi replay thật, rồi đọc lại CSV và ảnh kết quả.
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/replay_test_$<CONFIG>.cmake CONTENT "
set(root \"${CMAKE_CURRENT_BINARY_DIR}/replay_fixture\")
file(REMOVE_RECURSE \"\${root}\")
file(MAKE_DIRECTORY \"\${root}/run/masks\")
execute_process(COMMAND \"$<TARGET_FILE:test_lane_controller>\" --make-run \"\${root}/run\"
    RESULT_VARIABLE result TIMEOUT 5)
if(NOT result STREQUAL \"0\")
    message(FATAL_ERROR \"Không tạo được run giả: \${result}\")
endif()
execute_process(COMMAND \"$<TARGET_FILE:ucr_replay_control>\" --run \"\${root}/run\"
    --control \"${PROJECT_SOURCE_DIR}/control.yaml\" --out \"\${root}/out\"
    RESULT_VARIABLE result TIMEOUT 15)
if(NOT result STREQUAL \"0\")
    message(FATAL_ERROR \"Replay thất bại: \${result}\")
endif()
execute_process(COMMAND \"$<TARGET_FILE:test_lane_controller>\" --check-replay \"\${root}/out\"
    RESULT_VARIABLE result TIMEOUT 5)
if(NOT result STREQUAL \"0\")
    message(FATAL_ERROR \"CSV hoặc ảnh phủ sai: \${result}\")
endif()
configure_file(\"\${root}/run/control_log_legacy.csv\" \"\${root}/run/control_log.csv\" COPYONLY)
execute_process(COMMAND \"$<TARGET_FILE:ucr_replay_control>\" --run \"\${root}/run\"
    --control \"${PROJECT_SOURCE_DIR}/control.yaml\" --out \"\${root}/out_legacy\"
    RESULT_VARIABLE result TIMEOUT 15)
if(NOT result STREQUAL \"0\")
    message(FATAL_ERROR \"Replay log thiếu sự kiện thất bại: \${result}\")
endif()
execute_process(COMMAND \"$<TARGET_FILE:test_lane_controller>\" --check-replay-legacy \"\${root}/out_legacy\"
    RESULT_VARIABLE result TIMEOUT 5)
if(NOT result STREQUAL \"0\")
    message(FATAL_ERROR \"Replay log thiếu sự kiện đã tự mở chốt: \${result}\")
endif()
")
add_test(NAME control_replay COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/replay_test_$<CONFIG>.cmake)
set_tests_properties(control_params control_lane control_replay PROPERTIES LABELS "control" TIMEOUT 60)

# T55a: hợp đồng luật làn; golden chỉ sinh bằng lệnh riêng trên cây khung.
add_executable(test_t55_lane_rules test_t55_lane_rules.cpp)
target_link_libraries(test_t55_lane_rules PRIVATE ucr_control)
target_compile_definitions(test_t55_lane_rules PRIVATE T55_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
target_compile_options(test_t55_lane_rules PRIVATE -Wall -Wextra)
add_test(NAME acc_t55_mask COMMAND test_t55_lane_rules --check-mask)
set_tests_properties(acc_t55_mask PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
foreach(scenario disabled replay_b1 threshold below all_valid invalid_rows
        counter_reset clock_reset stale_reset replay_b1_8218 recovery disabled_empty)
    add_test(NAME acc_t55_${scenario} COMMAND test_t55_lane_rules ${scenario})
    set_tests_properties(acc_t55_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
add_test(NAME acc_t55_params COMMAND test_t55_lane_rules params ${CMAKE_CURRENT_BINARY_DIR}/t55_params.yaml)
set_tests_properties(acc_t55_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)

# T55c: B3 theo hợp đồng; khung 6100 chỉ chạy tay, chưa được xác nhận.
add_executable(test_t55_pair_rules test_t55_pair_rules.cpp)
target_link_libraries(test_t55_pair_rules PRIVATE ucr_control opencv_imgcodecs)
target_compile_definitions(test_t55_pair_rules PRIVATE T55_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
target_compile_options(test_t55_pair_rules PRIVATE -Wall -Wextra)
foreach(scenario off 6010 single_right single_left empty exact near_previous near_other near_none
        near_tie outside upper_gap upper_tie shift_limit near_invalid anchor_far_invalid reset stale invalid_reset width offset)
    add_test(NAME acc_t55_pair_${scenario} COMMAND test_t55_pair_rules ${scenario})
    set_tests_properties(acc_t55_pair_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
add_test(NAME acc_t55_pair_params COMMAND test_t55_pair_rules params ${CMAKE_CURRENT_BINARY_DIR}/t55_pair_params.yaml)
set_tests_properties(acc_t55_pair_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)

# T55d: chỉ thêm ca theo đặc tả; nhóm D chạy tay, chờ Quân.
foreach(scenario near_boundary_left near_boundary_right match_boundary_left match_boundary_right
        replay_b1_pair empty_mat_commands single_commands near_invalid_commands
        merge_mean merge_exact merge_chain merge_disabled merge_noise merge_off
        merge_near_outer merge_upper_roles merge_7675_930
        merge_default_gap_0p07 merge_upper_anchor_row1
        regress_A regress_B regress_C regress_total)
    add_test(NAME acc_t55_pair_${scenario} COMMAND test_t55_pair_rules ${scenario})
    set_tests_properties(acc_t55_pair_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
foreach(scenario params_order params_merge)
    add_test(NAME acc_t55_pair_${scenario} COMMAND test_t55_pair_rules ${scenario}
        ${CMAKE_CURRENT_BINARY_DIR}/t55_pair_${scenario}.yaml)
    set_tests_properties(acc_t55_pair_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()

# T56f: đạo hàm lọc theo đặc tả, dùng trường/khóa sẽ được DEV bổ sung.
add_executable(test_t56_kd_filter test_t56_kd_filter.cpp)
target_link_libraries(test_t56_kd_filter PRIVATE ucr_control)
target_compile_definitions(test_t56_kd_filter PRIVATE
    T56_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
    T56_CONTROL_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../../control.yaml")
target_compile_options(test_t56_kd_filter PRIVATE -Wall -Wextra)
foreach(scenario yaml_default step_noise off_exact lost stale reset ref recovery)
    add_test(NAME acc_t56_kd_${scenario} COMMAND test_t56_kd_filter ${scenario})
    set_tests_properties(acc_t56_kd_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
foreach(scenario golden params)
    add_test(NAME acc_t56_kd_${scenario} COMMAND test_t56_kd_filter ${scenario}
        ${CMAKE_CURRENT_BINARY_DIR}/t56_kd_${scenario}.yaml)
    set_tests_properties(acc_t56_kd_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()

add_executable(test_t66_obstacle test_t66_obstacle.cpp)
target_link_libraries(test_t66_obstacle PRIVATE ucr_control)
add_test(NAME control_t66_obstacle COMMAND test_t66_obstacle
    ${PROJECT_SOURCE_DIR}/control.yaml ${CMAKE_CURRENT_BINARY_DIR}/t66_params.yaml)
set_tests_properties(control_t66_obstacle PROPERTIES LABELS "control" TIMEOUT 60)

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner PRIVATE ucr_control)
add_test(NAME control_planner COMMAND test_planner)
set_tests_properties(control_planner PROPERTIES LABELS "control" TIMEOUT 15)

add_executable(test_t57_turn test_t57_turn.cpp)
target_link_libraries(test_t57_turn PRIVATE ucr_control)
target_compile_options(test_t57_turn PRIVATE -Wall -Wextra)
foreach(scenario signs votes j1 arm_distance yaw command disabled LOI_CAM_BIEN
        CAM_BIEN_CU sensor_invalid RE_QUA_CM RE_QUA_MS KET_RE red fault_red mask_stale
        low_conf_armed space_during_turn nonfinite_sensor
        armed_sensor_fault armed_red red_release_after_turn_red sign_while_turning after_turn_lane
        t68_signs t68_no_sign t68_red_hold t68_start_distance t68_arm_distance
        t68_conflict t68_avoidance t68_hold_heading)
    add_test(NAME acc_t57_${scenario} COMMAND test_t57_turn ${scenario})
    set_tests_properties(acc_t57_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
# Hồi quy: đang dừng vật cản thì không bắt đầu rẽ (ca unit, không thuộc bộ chấp nhận).
add_test(NAME test_t57_obstacle_stop_no_turn COMMAND test_t57_turn obstacle_stop_no_turn)
set_tests_properties(test_t57_obstacle_stop_no_turn PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME test_t74_barrier_turn COMMAND test_t57_turn barrier_turn)
set_tests_properties(test_t74_barrier_turn PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME test_t68b_roundabout COMMAND test_t57_turn roundabout)
set_tests_properties(test_t68b_roundabout PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME test_t77_hold_stall COMMAND test_t57_turn roundabout_stall)
set_tests_properties(test_t77_hold_stall PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME test_t73_fault_recovers COMMAND test_t57_turn fault_recovers)
set_tests_properties(test_t73_fault_recovers PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME test_t73_fault_drops_rb COMMAND test_t57_turn roundabout_fault_drops_pending)
set_tests_properties(test_t73_fault_drops_rb PROPERTIES LABELS "control" TIMEOUT 60)
add_test(NAME acc_t57_params COMMAND test_t57_turn params ${CMAKE_CURRENT_BINARY_DIR}/t57_params.yaml)
set_tests_properties(acc_t57_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
add_test(NAME acc_t57_t68_params COMMAND test_t57_turn t68_params ${CMAKE_CURRENT_BINARY_DIR}/t68_params.yaml)
set_tests_properties(acc_t57_t68_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)

# Lô 3b: chấp nhận Planner thuần theo T74/T68b/T77-B/T75, không phụ thuộc phần cứng.
add_executable(test_t74_t77 test_t74_t77.cpp)
target_link_libraries(test_t74_t77 PRIVATE ucr_control)
target_compile_options(test_t74_t77 PRIVATE -Wall -Wextra)
foreach(scenario thresholds votes approach turn_yaw box_moves no_repeat space_go sign_priority sensor_fault disabled)
    add_test(NAME acc_t74_${scenario} COMMAND test_t74_t77 t74_${scenario})
    set_tests_properties(acc_t74_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
add_test(NAME acc_t74_params COMMAND test_t74_t77 t74_params ${CMAKE_CURRENT_BINARY_DIR}/t74_lot3b_params.yaml)
set_tests_properties(acc_t74_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
foreach(scenario pass1_distance pass2_distance flicker after_two visible sign_priority votes sensor_fault zero_distance)
    add_test(NAME acc_t68b_${scenario} COMMAND test_t74_t77 t68b_${scenario})
    set_tests_properties(acc_t68b_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
add_test(NAME acc_t68b_params COMMAND test_t74_t77 t68b_params ${CMAKE_CURRENT_BINARY_DIR}/t68b_lot3b_params.yaml)
set_tests_properties(acc_t68b_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
foreach(scenario only_on_loss yaw_sign wrap steer_max steer_rate speed regain pass_end hold_distance sensor_entry sensor_mid stale_mask stale_image space_go reset block_turn block_barrier block_obstacle red person)
    add_test(NAME acc_t77_${scenario} COMMAND test_t74_t77 t77_${scenario})
    set_tests_properties(acc_t77_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
add_test(NAME acc_t77_params COMMAND test_t74_t77 t77_params ${CMAKE_CURRENT_BINARY_DIR}/t77_lot3b_params.yaml)
set_tests_properties(acc_t77_params PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
foreach(scenario people person votes low_conf release absent turn_pause_stuck turn_pause_timeout space_priority)
    add_test(NAME acc_t75_${scenario} COMMAND test_t74_t77 t75_${scenario})
    set_tests_properties(acc_t75_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()

foreach(scenario action0 recover_barrier_moved fault_while_barrier_turn vs_red startup_fault_barrier)
    add_test(NAME acc_q7_${scenario} COMMAND test_t74_t77 q7_${scenario})
    set_tests_properties(acc_q7_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
foreach(scenario sign_blocked_until_end speed_min space_in_ve_phai pass_cm_scale lane_lost)
    add_test(NAME acc_q8_${scenario} COMMAND test_t57_turn q8_${scenario})
    set_tests_properties(acc_q8_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
foreach(scenario flicker_resets_clock one_source_stale rb_not_revived)
    add_test(NAME acc_q9_${scenario} COMMAND test_t74_t77 q9_${scenario})
    set_tests_properties(acc_q9_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()

add_executable(test_t58_vote_release test_t58_vote_release.cpp)
target_link_libraries(test_t58_vote_release PRIVATE ucr_control)
foreach(scenario hold_2of5 release_2of5 hold_1of5 start_2of5 default_3of5 stale_red
        start_green_only repeat_batch window1 even_2of4 mixed_batch stop_while_waiting relatch_after_release)
    add_test(NAME acc_t58_${scenario} COMMAND test_t58_vote_release ${scenario})
    set_tests_properties(acc_t58_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()

add_executable(test_t59_pair_hold test_t59_pair_hold.cpp)
target_link_libraries(test_t59_pair_hold PRIVATE ucr_control)
target_compile_definitions(test_t59_pair_hold PRIVATE T55_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
target_compile_options(test_t59_pair_hold PRIVATE -Wall -Wextra)
foreach(scenario jump_left straight_stable straight_seq far_not_worse no_row_loss flip pair0_golden)
    add_test(NAME acc_t59_${scenario} COMMAND test_t59_pair_hold ${scenario})
    set_tests_properties(acc_t59_${scenario} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
