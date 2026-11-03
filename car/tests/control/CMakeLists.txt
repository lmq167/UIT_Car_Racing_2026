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

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner PRIVATE ucr_control)
add_test(NAME control_planner COMMAND test_planner)
set_tests_properties(control_planner PROPERTIES LABELS "control" TIMEOUT 15)
