# T33: phiên TEST sở hữu file này; DEV chỉ include trong BUILD_TESTING.
if(NOT BUILD_TESTING)
    return()
endif()
find_package(PythonInterp 3.6 REQUIRED)
if(NOT TARGET ucr_collect_drive_hooks OR NOT TARGET ucr_collect_drive)
    message(FATAL_ERROR "Thieu target ucr_collect_drive/hooks theo hop dong T33")
endif()
add_executable(test_collect_drive_limits "${CMAKE_CURRENT_LIST_DIR}/test_limits.cpp")
set_target_properties(test_collect_drive_limits PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_collect_drive_limits PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME acc_collect_drive_L01 COMMAND test_collect_drive_limits)
set_tests_properties(acc_collect_drive_L01 PROPERTIES LABELS "acceptance;unit" TIMEOUT 15)
# Tự kiểm bộ đọc vết/bộ vá mutation trên mã mẫu; không chạy binary.
add_test(NAME acc_collect_drive_SC
    COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_collect_drive.py" --case SC)
set_tests_properties(acc_collect_drive_SC PROPERTIES LABELS "acceptance;unit" TIMEOUT 30)
execute_process(COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE _collect_revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET TIMEOUT 2)
if(NOT _collect_revision)
    set(_collect_revision "unknown")
endif()
# Như drive_collect: +300 ms chỉ với sanitizer cho hạn dừng mất ảnh; Release giữ trần gốc.
set(_collect_slack 0)
set(_collect_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_RELEASE}")
if(_collect_flags MATCHES "sanitize" OR UCR_ENABLE_ASAN OR UCR_ENABLE_TSAN)
    set(_collect_slack 300)
endif()
set(_collect_cases K01 K02 K03 K04 K05 S01 E01 E02 E02b E03 E03b E04 E05 E05b E06 E06b E07 E09 E10
    P01 P02 P03 F01 F02 F03 F03b F04 M1 M1b M2 M2s M3 M4 M5)
foreach(_collect_case IN LISTS _collect_cases)
    add_test(NAME acc_collect_drive_${_collect_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_collect_drive.py"
        --case "${_collect_case}" --hooks "$<TARGET_FILE:ucr_collect_drive_hooks>"
        --real "$<TARGET_FILE:ucr_collect_drive>"
        --limits "$<TARGET_FILE:test_collect_drive_limits>"
        --source "${CMAKE_CURRENT_LIST_DIR}" --cmake "${CMAKE_COMMAND}"
        --cxx "${CMAKE_CXX_COMPILER}" --nm "${CMAKE_NM}"
        --revision "${_collect_revision}" --slack "${_collect_slack}")
    set(_collect_labels "acceptance;threads")
    set(_collect_timeout 120)
    if(_collect_case MATCHES "^M")
        # Baseline + build bản sao -j1 (M2/M2s dựng 3 đích) + ca dò.
        set(_collect_labels "acceptance;slow")
        set(_collect_timeout 300)
    endif()
    set_tests_properties(acc_collect_drive_${_collect_case} PROPERTIES
        LABELS "${_collect_labels}" TIMEOUT ${_collect_timeout})
endforeach()
