# Biến đường dẫn binary hooks: UCR_AUTO_HOOKS_BINARY (mặc định $<TARGET_FILE:ucr_auto_hooks>).
# Phiên TEST sở hữu; DEV include file này từ tools/auto_drive/CMakeLists.txt.
if(NOT BUILD_TESTING)
    return()
endif()
find_package(PythonInterp 3.6 REQUIRED)
if(NOT DEFINED UCR_AUTO_HOOKS_BINARY)
    if(NOT TARGET ucr_auto_hooks)
        message(FATAL_ERROR "Thieu target ucr_auto_hooks theo hop dong T47")
    endif()
    set(UCR_AUTO_HOOKS_BINARY "$<TARGET_FILE:ucr_auto_hooks>")
endif()
# Giống hệt collect_drive/tests.cmake: chỉ sanitizer cộng 300ms, Release cộng0.
set(_auto_slack 0)
set(_auto_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_RELEASE}")
if(_auto_flags MATCHES "sanitize" OR UCR_ENABLE_ASAN OR UCR_ENABLE_TSAN)
    set(_auto_slack 300)
endif()
set(_auto_cases A01 A02 A03 A04 A05 A06 A07 A08 A09 A10 A11 A12 A13 A14 A15 A16 A17
    A18 A19 A20 A21 A22 A23 A24 A25 A26 A27 A28 A29 A30 A30c A30d A30e A30f A30g A30h A31 A32 A33 A34 A35
    A40 A41 A42 A43 A44 A45 A46 A47 A48 A07b A08b A40b A42b A43b A49 A50 A51
    A70u A70y A71u A71y A72u A72y A73 A74ua A74ub A74ya A74yb A75 A77 A78 A79 A80 A81 A60 B5
    t57_disabled_equivalence t57_open_imu t57_open_imu_drift t57_read_imu t57_read_encoder t57_reload t57_stale_imu t57_exit_turning
    t57_lost_image_turning t57_space_during_turn t57_g_pending_after_fault
    t57_reload_during_turn t57_init_ok_order
    t57_transient_imu_armed t57_transient_encoder_armed
    t57_transient_imu_turning t57_transient_encoder_turning
    t57_transient_idle t57_csv_imu t57_csv_encoder t57_csv_both
    t71_defaults t71_steer_40 t71_steer_40_1 t71_cruise_15 t71_cruise_16 t71_motor_cap t71_curve_30
    t72_zero_valid t72_negative_rejected t72_zero_keeps_text t72_reload_0_to_5
    t73_default_enable t73_init_imu_fail t73_init_drift_fail t73_reload_no_imu
    t73_imu_turning t73_encoder_turning t73_stale_imu_turning t73_stale_encoder_turning
    t73_imu_lane t73_encoder_lane t73_g_reopens_dung t73_lost_image_turning t73_stale_mask_turning)
foreach(_auto_case IN LISTS _auto_cases)
    add_test(NAME acc_auto_${_auto_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_auto_drive.py"
        --case "${_auto_case}" --hooks "${UCR_AUTO_HOOKS_BINARY}" --slack "${_auto_slack}")
    set_tests_properties(acc_auto_${_auto_case} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
endforeach()
add_test(NAME auto_runs_gitignore
    COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_auto_drive.py" --case gitignore)
set_tests_properties(auto_runs_gitignore PROPERTIES TIMEOUT 30)

foreach(_auto_preview_case P01 P02)
    add_test(NAME acc_auto_preview_${_auto_preview_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_auto_preview.py"
        --case "${_auto_preview_case}" --hooks "${UCR_AUTO_HOOKS_BINARY}" --slack "${_auto_slack}")
    set_tests_properties(acc_auto_preview_${_auto_preview_case} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
endforeach()

foreach(_auto_debug_case D01 D02)
    add_test(NAME acc_auto_debug_${_auto_debug_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_auto_debug.py"
        --case "${_auto_debug_case}" --hooks "${UCR_AUTO_HOOKS_BINARY}" --slack "${_auto_slack}")
    set_tests_properties(acc_auto_debug_${_auto_debug_case} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
endforeach()
