# T21b_design §1–§10; PTY/khóa/vết riêng, chỉ chạy con giả của bản hooks.
if(NOT BUILD_TESTING)
    return()
endif()
find_package(PythonInterp 3.6 REQUIRED)
if(NOT TARGET ucr_drive_collect_hooks OR NOT TARGET ucr_drive_collect)
    message(FATAL_ERROR "Thieu dich drive/hooks cho test preview")
endif()
set(_preview_slack 0)
set(_preview_tsan_slack 0)
set(_preview_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_RELEASE}")
if(_preview_flags MATCHES "sanitize" OR UCR_ENABLE_ASAN OR UCR_ENABLE_TSAN)
    set(_preview_slack 300)
endif()
if(_preview_flags MATCHES "sanitize=[^ ]*thread" OR UCR_ENABLE_TSAN)
    set(_preview_tsan_slack 1000)
endif()
set(_preview_cam "${CMAKE_BINARY_DIR}/tools/cam_collect/ucr_collect_cam")
if(TARGET ucr_collect_cam)
    set(_preview_cam "$<TARGET_FILE:ucr_collect_cam>")
endif()
# P20 cần giao diện X: chưa có chế độ giả thu nhỏ/mất focus được công bố.
set(_preview_cases P01 P02 P03 P04 P05 P06 P07 P08 P09 P10 P11 P12 P13 P14 P15 P16 P17 P18
    P19 P21 P22 P23)
foreach(_preview_case IN LISTS _preview_cases)
    add_test(NAME acc_drive_preview_${_preview_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_preview.py"
        --case "${_preview_case}" --hooks "$<TARGET_FILE:ucr_drive_collect_hooks>"
        --real "$<TARGET_FILE:ucr_drive_collect>" --cam "${_preview_cam}"
        --build-root "${CMAKE_BINARY_DIR}" --nm "${CMAKE_NM}"
        --source "${CMAKE_CURRENT_LIST_DIR}" --cmake "${CMAKE_COMMAND}"
        --cxx "${CMAKE_CXX_COMPILER}"
        --slack "${_preview_slack}" --tsan-slack "${_preview_tsan_slack}")
    set(_preview_labels "acceptance;threads")
    if(_preview_case MATCHES "^P(16|17|18)$")
        set(_preview_labels "acceptance;slow")
    endif()
    # P23 (so số ảnh có/không preview): nhịp lưu dao động bất thường dưới TSan nên phép so tốc độ không có nghĩa; vẫn chạy ở ASan + container.
    # P13 (con treo + stderr đầy, hạn cleanup 3 s): như D86/D88, dưới TSan tiến trình treo ở đường _exit khi pipe stderr đầy;
    # giới hạn của TSan, không phải của mã xe. Ca vẫn chạy ở ASan và container GCC 7.
    if(_preview_case STREQUAL "P13" OR _preview_case STREQUAL "P23")
        set(_preview_labels "acceptance")
    endif()
    if(_preview_case STREQUAL "P19")
        set(_preview_labels "acceptance;threads;slow")
    endif()
    set_tests_properties(acc_drive_preview_${_preview_case} PROPERTIES
        LABELS "${_preview_labels}" TIMEOUT 120)
endforeach()
