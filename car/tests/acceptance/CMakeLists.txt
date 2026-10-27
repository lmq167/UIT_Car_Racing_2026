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
    # Không biến lỗi thiết lập/mã 77 thành bỏ qua ở nhãn acceptance.
    set_tests_properties(${t} PROPERTIES LABELS "acceptance" TIMEOUT 120)
endforeach()
# Ca cũ gộp cả so sánh đúng biên ToF: giữ chạy hồi quy, không dùng làm cổng acceptance của phanh ToF (QĐ 2).
set_tests_properties(acc_safety_logic PROPERTIES LABELS "regression")
# ASan nền 27/10: acc_cli mất 30,38 s; giữ nguyên các nhãn và hạn giờ đã có.
set_property(TEST acc_cli APPEND PROPERTY LABELS slow)

# --- Kiểm tra lệnh phần cứng giả qua vết UCR_FAKE_HW_TRACE, ToF giả, ngắt camera, lỗi giả, App::SetController.
# Chỉ có ở bản giả lập (các điểm cắm không tồn tại ở UCR_FAKE_HW=OFF).
if(UCR_FAKE_HW)
    add_executable(acc_seed_gate acc_seed_gate.cpp)
    target_link_libraries(acc_seed_gate PRIVATE app_core)
    target_compile_options(acc_seed_gate PRIVATE -Wall -Wextra)
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
        set_tests_properties(acc_hw_${scenario} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
    endforeach()
    set_tests_properties(acc_hw_tof_boundary PROPERTIES LABELS "regression;threads")
    add_test(NAME acc_hwtrace_sh COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/acc_hwtrace.sh
        $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> $<TARGET_FILE:acc_sigrun>)
    set_tests_properties(acc_hwtrace_sh PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
    # ASan nền 27/10: acc_hwtrace_sh mất 50,24 s.
    set_property(TEST acc_hwtrace_sh APPEND PROPERTY LABELS slow)

    # Hiệu chuẩn/an toàn: giao diện đã duyệt; không đăng ký các mục H/M01 và không có mã bỏ qua.
    find_package(PythonInterp 3.6 REQUIRED)
    set(ACC_VEHICLE_CLI ${CMAKE_CURRENT_SOURCE_DIR}/acc_vehicle_cli.py)
    # Python 3.6 trong container có thể dùng locale ASCII.
    set(ACC_VEHICLE_PYTHON ${CMAKE_COMMAND} -E env PYTHONIOENCODING=utf-8 ${PYTHON_EXECUTABLE})
    add_executable(acc_recovery_timing acc_recovery_timing.cpp)
    target_link_libraries(acc_recovery_timing PRIVATE Threads::Threads)
    target_compile_options(acc_recovery_timing PRIVATE -Wall -Wextra)
    add_test(NAME acc_recovery_timing_selfcheck COMMAND acc_recovery_timing)
    set_tests_properties(acc_recovery_timing_selfcheck PROPERTIES LABELS "acceptance" TIMEOUT 10)
    add_executable(acc_safety_components acc_safety_components.cpp)
    target_link_libraries(acc_safety_components PRIVATE app_core)
    target_compile_options(acc_safety_components PRIVATE -Wall -Wextra)
    foreach(scenario config rear recovery servo smoke_result calib_safety smoke_sensors smoke_servo)
        add_test(NAME acc_${scenario} COMMAND acc_safety_components ${scenario})
        set_tests_properties(acc_${scenario} PROPERTIES LABELS "acceptance" TIMEOUT 30)
    endforeach()
    add_executable(acc_recovery_app acc_recovery_app.cpp)
    target_link_libraries(acc_recovery_app PRIVATE app_core)
    target_compile_options(acc_recovery_app PRIVATE -Wall -Wextra)
    foreach(scenario short below above stable_changed stable_default twice long_short short_long clamp tof lower reverse direction startup config_changed gate gate_missing)
        add_test(NAME acc_recovery_${scenario} COMMAND acc_recovery_app ${scenario})
        set_tests_properties(acc_recovery_${scenario} PROPERTIES LABELS "acceptance;threads" TIMEOUT 30)
    endforeach()
    # Cùng main.cpp thật, chỉ thay bộ điều khiển qua API công khai trước Start; không thêm cờ cho xe.
    add_executable(acc_recovery_main ${PROJECT_SOURCE_DIR}/main.cpp acc_recovery_inject.cpp)
    target_link_libraries(acc_recovery_main PRIVATE app_core)
    target_compile_definitions(acc_recovery_main PRIVATE UCR_ENV_CHECK_SCRIPT="${PROJECT_SOURCE_DIR}/scripts/env_check.sh")
    target_compile_options(acc_recovery_main PRIVATE -Wall -Wextra)
    set_property(TARGET acc_recovery_main APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--wrap=_ZN3app3App5StartEv")
    foreach(state MAT_ANH DUNG_CHO_ANH_ON_DINH)
        foreach(sig INT TERM)
            add_test(NAME acc_recovery_auto_${state}_${sig} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} auto_signal
                ${PROJECT_SOURCE_DIR} $<TARGET_FILE:acc_recovery_main> $<TARGET_FILE:ucr_stop> ${state} ${sig})
            set_tests_properties(acc_recovery_auto_${state}_${sig} PROPERTIES LABELS "acceptance;threads" TIMEOUT 45)
        endforeach()
    endforeach()
    foreach(state MAT_ANH DUNG_CHO_ANH_ON_DINH)
        add_test(NAME acc_recovery_auto_keys_${state} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} auto_keys
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:acc_recovery_main> $<TARGET_FILE:ucr_stop> ${state})
        set_tests_properties(acc_recovery_auto_keys_${state} PROPERTIES LABELS "acceptance;threads" TIMEOUT 90)
    endforeach()
    foreach(mode smoke calib collect)
        add_test(NAME acc_no_gate_${mode} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} no_gate
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> ${mode})
        set_tests_properties(acc_no_gate_${mode} PROPERTIES LABELS "acceptance;threads" TIMEOUT 45)
    endforeach()
    foreach(scenario check_config_valid check_config_types check_config_ranges check_config_errors)
        add_test(NAME acc_${scenario} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} ${scenario}
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop>)
        set_tests_properties(acc_${scenario} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
    endforeach()
    foreach(scenario defaults config_numeric config_keys wrapper_config cli smoke_unconfirmed smoke_gate env collect_print collect_cancel)
        add_test(NAME acc_cli_${scenario} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} ${scenario}
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop>)
        set_tests_properties(acc_cli_${scenario} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
    endforeach()
    # D01–D06 chạy trên laptop bằng tools/check_calib_docs.py; runner chỉ chép mã xe.
    foreach(step env servo radius motor position imu tof camera latency timing)
        add_test(NAME acc_calib_${step} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} calib
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> ${step} TERM)
        set_tests_properties(acc_calib_${step} PROPERTIES LABELS "acceptance;threads" TIMEOUT 45)
    endforeach()
    add_test(NAME acc_calib_int COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} calib
        ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> motor INT)
    set_tests_properties(acc_calib_int PROPERTIES LABELS "acceptance;threads" TIMEOUT 45)
    foreach(sig INT TERM)
        add_test(NAME acc_smoke_${sig} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} smoke_signal
            ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> ${sig})
        set_tests_properties(acc_smoke_${sig} PROPERTIES LABELS "acceptance;threads" TIMEOUT 45)
    endforeach()
    foreach(mode calib smoke)
        foreach(fault actuator_throw sensor_throw terminate)
            add_test(NAME acc_${mode}_${fault} COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} fault
                ${PROJECT_SOURCE_DIR} $<TARGET_FILE:main> $<TARGET_FILE:ucr_stop> ${mode} ${fault})
            set_tests_properties(acc_${mode}_${fault} PROPERTIES LABELS "acceptance;threads" TIMEOUT 60)
        endforeach()
    endforeach()
    add_test(NAME acc_harness_selfcheck COMMAND ${ACC_VEHICLE_PYTHON} ${ACC_VEHICLE_CLI} self-test)
    set_tests_properties(acc_harness_selfcheck PROPERTIES LABELS "acceptance" TIMEOUT 10)
endif()
