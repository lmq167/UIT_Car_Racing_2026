# Test chấp nhận (acceptance test) – nhãn ctest "acceptance". Viết độc lập từ yêu cầu an toàn và tài liệu cấu hình;
# Test chấp nhận độc lập, nhãn ctest: acceptance.
# Được nạp từ CMakeLists.txt gốc, trong khối if(BUILD_TESTING). Yêu cầu CMake >= 3.10, C++17, OpenCV >= 4.1.

# --- Logic an toàn + mặc định cấu hình (đơn vị, không cần tiến trình ngoài)
add_executable(acc_safety_logic acc_safety_logic.cpp)
target_link_libraries(acc_safety_logic PRIVATE app_core)
add_test(NAME acc_safety_logic COMMAND acc_safety_logic)

# --- Lớp App ở UCR_FAKE_HW=ON: thoát bình thường, huỷ khi đang chạy, ngoại lệ, giới hạn lệnh, mất ảnh (đầu-cuối qua log.csv)
add_executable(acc_app_lifecycle acc_app_lifecycle.cpp)
target_link_libraries(acc_app_lifecycle PRIVATE app_core)
add_test(NAME acc_app_lifecycle COMMAND acc_app_lifecycle)

add_executable(acc_app_fault acc_app_fault.cpp)
target_link_libraries(acc_app_fault PRIVATE app_core)
foreach(scenario throw clamp_pos clamp_neg nan pass imgloss)
    add_test(NAME acc_app_fault_${scenario} COMMAND acc_app_fault ${scenario})
endforeach()

# --- Lớp Motor thật qua UART giả (pty): chuỗi init đúng 1 lần, ucr_stop không gửi lệnh cấu hình, lệnh tốc độ không chờ.
# Biên dịch riêng Motor.cpp + Logger.cpp (không phụ thuộc I2C), không liên kết app_core để tránh trùng ký hiệu.
add_executable(acc_motor_uart acc_motor_uart.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/Motor.cpp
    ${PROJECT_SOURCE_DIR}/src/common/Logger.cpp)
target_include_directories(acc_motor_uart PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(acc_motor_uart PRIVATE Threads::Threads)
add_test(NAME acc_motor_uart COMMAND acc_motor_uart)

# --- Công cụ tạo ảnh giả + các test dòng lệnh / script
add_executable(acc_sigrun acc_sigrun.cpp)
add_executable(acc_make_frames acc_make_frames.cpp)
target_link_libraries(acc_make_frames PRIVATE app_core)

add_test(NAME acc_cli COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/acc_cli.sh
    $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> $<TARGET_FILE:acc_make_frames> $<TARGET_FILE:acc_sigrun>)
add_test(NAME acc_run_sh COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/acc_run_sh.sh)
add_test(NAME acc_signals COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/acc_signals.sh
    $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> $<TARGET_FILE:acc_sigrun>)

set(ACC_TESTS acc_safety_logic acc_app_lifecycle acc_app_fault_throw acc_app_fault_clamp_pos acc_app_fault_clamp_neg
    acc_app_fault_nan acc_app_fault_pass acc_app_fault_imgloss acc_motor_uart acc_cli acc_run_sh acc_signals)
foreach(t ${ACC_TESTS})
    # Mã 77 = CHƯA KIỂM ĐƯỢC (thiếu giao diện/pty): ctest báo "Skipped/Not Run", không phải đạt.
    set_tests_properties(${t} PROPERTIES LABELS "acceptance" TIMEOUT 180 SKIP_RETURN_CODE 77)
endforeach()

# --- Kiểm tra lệnh phần cứng giả qua vết UCR_FAKE_HW_TRACE, ToF giả, ngắt camera, lỗi giả, App::SetController.
# Chỉ có ở bản giả lập (các điểm cắm không tồn tại ở UCR_FAKE_HW=OFF).
if(UCR_FAKE_HW)
    add_executable(acc_hwtrace acc_hwtrace.cpp)
    target_link_libraries(acc_hwtrace PRIVATE app_core)
    set(ACC_HW_SCENARIOS
        servo_trim servo_invert servo_edge_pos servo_edge_neg
        tail_normal tail_destroy tail_ctl_throw tail_actuator_throw tail_sensor_throw
        keepalive_100 keepalive_200 keepalive_steer speed_change init_once
        tof_brake tof_invalid tof_brake_slow tof_reverse tof_rear tof_order tof_boundary tof_start_invalid
        camera_gap camera_gap_200)
    foreach(scenario ${ACC_HW_SCENARIOS})
        add_test(NAME acc_hw_${scenario} COMMAND acc_hwtrace ${scenario})
        # Mã 77 = CHƯA KIỂM ĐƯỢC; thời gian chờ rộng vì máy chậm / container.
        set_tests_properties(acc_hw_${scenario} PROPERTIES LABELS "acceptance" TIMEOUT 120 SKIP_RETURN_CODE 77)
    endforeach()
    add_test(NAME acc_hwtrace_sh COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/acc_hwtrace.sh
        $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> $<TARGET_FILE:acc_sigrun>)
    set_tests_properties(acc_hwtrace_sh PROPERTIES LABELS "acceptance" TIMEOUT 400 SKIP_RETURN_CODE 77)
endif()
