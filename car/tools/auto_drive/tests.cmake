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
    A18 A19 A20 A21 A22 A23 A24 A25 A26 A27 A28 A29 A31 A32 A33 A34 A35 B5)
foreach(_auto_case IN LISTS _auto_cases)
    add_test(NAME acc_auto_${_auto_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_auto_drive.py"
        --case "${_auto_case}" --hooks "${UCR_AUTO_HOOKS_BINARY}" --slack "${_auto_slack}")
    set_tests_properties(acc_auto_${_auto_case} PROPERTIES LABELS "acceptance;threads" TIMEOUT 120)
endforeach()
