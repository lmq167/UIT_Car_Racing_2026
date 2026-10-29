# T21/T24: 87 ca D + 7 ca M; PTY/thư mục/khóa/vết riêng.
# DEV include file này trong BUILD_TESTING; không sửa CMakeLists.txt của DEV.
if(NOT BUILD_TESTING)
    return()
endif()
find_package(PythonInterp 3.6 REQUIRED)
if(NOT TARGET ucr_drive_collect_hooks OR NOT TARGET ucr_drive_collect)
    message(FATAL_ERROR "Thieu binary drive/hooks de dang ky test chap nhan")
endif()

set(_drive_slack 0)
set(_drive_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_RELEASE}")
if(_drive_flags MATCHES "sanitize" OR UCR_ENABLE_ASAN OR UCR_ENABLE_TSAN)
    # §8 cho phép thêm 300 ms duy nhất với sanitizer; Release giữ trần gốc.
    set(_drive_slack 300)
endif()
set(_drive_tsan_slack 0)
if(_drive_flags MATCHES "sanitize=[^ ]*thread" OR UCR_ENABLE_TSAN)
    # Chỉ dung sai khung đầu 5 s và cleanup 3 s; giữ nguyên các mốc an toàn.
    set(_drive_tsan_slack 1000)
endif()
set(_drive_cases
    D01 D02 D03 D06 D07 D08 D09 D10 D11 D12 D13 D14 D15 D16
    D17 D18 D19 D20 D21 D22 D23 D24 D25 D26 D27 D28 D29 D30 D31 D32
    D33 D34 D35 D36 D37 D38 D39 D40 D41 D43 D44 D45 D46 D47 D48 D49
    D50 D51 D53 D55 D56 D57 D58 D60 D62
    D65 D66 D67 D68 D69 D70 D71 D72 D73 D74 D75 D76 D77 D78 D79
    D80 D81 D82 D83 D84
    D85 D86 D87 D88 D89 D90 D91 D92 D93 D94 D95 D96
    M01 M02 M03 M04 M05 M06 M07)
set(_drive_cam "${CMAKE_BINARY_DIR}/tools/cam_collect/ucr_collect_cam")
if(TARGET ucr_collect_cam)
    set(_drive_cam "$<TARGET_FILE:ucr_collect_cam>")
endif()
foreach(_drive_case IN LISTS _drive_cases)
    set(_drive_case_tsan_slack ${_drive_tsan_slack})
    if(_drive_tsan_slack AND _drive_case MATCHES "^D(26|81|86|88|91)$")
        # Chỉ tăng thời gian quan sát thoát của fix8; D87 và các ca khác giữ nguyên.
        set(_drive_case_tsan_slack 2500)
    endif()
    add_test(NAME acc_drive_collect_${_drive_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_drive_collect.py"
        --case "${_drive_case}"
        --hooks "$<TARGET_FILE:ucr_drive_collect_hooks>"
        --real "$<TARGET_FILE:ucr_drive_collect>" --cam "${_drive_cam}"
        --source "${CMAKE_CURRENT_LIST_DIR}"
        --wrapper "${CMAKE_CURRENT_LIST_DIR}/../../scripts/run_drive.sh"
        --cmake "${CMAKE_COMMAND}" --cxx "${CMAKE_CXX_COMPILER}"
        --slack "${_drive_slack}" --tsan-slack "${_drive_case_tsan_slack}")
    set(_drive_labels "acceptance;threads")
    if(_drive_case MATCHES "^M")
        set(_drive_labels "acceptance;slow")
    endif()
    # D86/D88 (stderr nghẽn, hẹn giờ cứng 3 s rồi _exit): dưới TSan tiến trình treo ở đường _exit khi pipe stderr
    # đầy (đo riêng: ASan thoát 3,1 s, TSan quá 20 s). Đây là giới hạn của TSan, không phải của mã xe;
    # hai ca vẫn chạy ở ASan và container GCC 7 (Release).
    if(_drive_case STREQUAL "D86" OR _drive_case STREQUAL "D88")
        set(_drive_labels "acceptance")
    endif()
    set_tests_properties(acc_drive_collect_${_drive_case} PROPERTIES
        LABELS "${_drive_labels}" TIMEOUT 120)
endforeach()
