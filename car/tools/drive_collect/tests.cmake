# T21_design §6–§13: 78 ca D + 5 ca M = 83; mỗi ca có PTY/thư mục/khóa/vết riêng.
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
set(_drive_cases
    D01 D02 D03 D04 D05 D06 D07 D08 D09 D10 D11 D12 D13 D14 D15 D16
    D17 D18 D19 D20 D21 D22 D23 D24 D25 D26 D27 D28 D29 D30 D31 D32
    D33 D34 D35 D36 D37 D38 D39 D40 D41 D43 D44 D45 D46 D47 D48 D49
    D50 D51 D53 D55 D56 D57 D58 D59 D60 D62
    D65 D66 D67 D68 D69 D70 D71 D72 D73 D74 D75 D76 D77 D78 D79
    D80 D81 D82 D83 D84
    M01 M02 M03 M04 M05)
set(_drive_cam "${CMAKE_BINARY_DIR}/tools/cam_collect/ucr_collect_cam")
if(TARGET ucr_collect_cam)
    set(_drive_cam "$<TARGET_FILE:ucr_collect_cam>")
endif()
foreach(_drive_case IN LISTS _drive_cases)
    add_test(NAME acc_drive_collect_${_drive_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_drive_collect.py"
        --case "${_drive_case}"
        --hooks "$<TARGET_FILE:ucr_drive_collect_hooks>"
        --real "$<TARGET_FILE:ucr_drive_collect>" --cam "${_drive_cam}"
        --source "${CMAKE_CURRENT_LIST_DIR}"
        --wrapper "${CMAKE_CURRENT_LIST_DIR}/../../scripts/run_drive.sh"
        --cmake "${CMAKE_COMMAND}" --cxx "${CMAKE_CXX_COMPILER}"
        --slack "${_drive_slack}")
    set(_drive_labels "acceptance;threads")
    if(_drive_case MATCHES "^M")
        set(_drive_labels "acceptance;slow")
    endif()
    set_tests_properties(acc_drive_collect_${_drive_case} PROPERTIES
        LABELS "${_drive_labels}" TIMEOUT 120)
endforeach()
