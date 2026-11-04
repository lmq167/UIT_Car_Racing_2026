add_executable(test_control_params test_control_params.cpp)
target_link_libraries(test_control_params PRIVATE ucr_control)
add_test(NAME control_params COMMAND test_control_params
    ${PROJECT_SOURCE_DIR}/control.yaml ${CMAKE_CURRENT_BINARY_DIR}/params_test.yaml)
add_executable(test_lane_controller test_lane_controller.cpp)
target_link_libraries(test_lane_controller PRIVATE ucr_control opencv_imgcodecs)
add_test(NAME control_lane COMMAND test_lane_controller)

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
        near_tie outside upper_gap upper_tie shift_limit near_invalid reset stale invalid_reset width offset)
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
