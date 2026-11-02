# Test ucr_test_model: dùng lại nguyên bộ chấp nhận T33 (nạp test_collect_drive.py làm module) + ca model.
if(NOT BUILD_TESTING)
    return()
endif()
find_package(PythonInterp 3.6 REQUIRED)
get_filename_component(_tm_tests "${CMAKE_CURRENT_LIST_DIR}/../../tests" ABSOLUTE)
foreach(_tm_unit core worker dnn)
    add_executable(test_model_view_${_tm_unit} "${_tm_tests}/test_model_view_${_tm_unit}.cpp")
    target_include_directories(test_model_view_${_tm_unit} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/model_view")
    target_link_libraries(test_model_view_${_tm_unit} PRIVATE test_model_view)
endforeach()
add_test(NAME test_model_view_core COMMAND test_model_view_core WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME test_model_view_worker COMMAND test_model_view_worker)
set_tests_properties(test_model_view_core PROPERTIES LABELS "unit" TIMEOUT 30)
set_tests_properties(test_model_view_worker PROPERTIES LABELS "unit;threads" TIMEOUT 60)
add_executable(test_test_model_limits "${CMAKE_CURRENT_LIST_DIR}/test_limits.cpp")
set_target_properties(test_test_model_limits PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE _tm_revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET TIMEOUT 2)
if(NOT _tm_revision)
    set(_tm_revision "unknown")
endif()
set(_tm_slack 0)
set(_tm_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_RELEASE}")
if(_tm_flags MATCHES "sanitize" OR UCR_ENABLE_ASAN OR UCR_ENABLE_TSAN)
    set(_tm_slack 300)
endif()
# Ca T33 (cùng mã) chạy trên binary mới; ca A/D/M6-M9 là phần model.
set(_tm_cases K01 K02 K03 K04 K05 S01 E01 E02 E02b E03 E03b E04 E05 E05b E06 E06b E07 E09 E10
    P01 P02 P03 F01 F02 F03 F03b F04 M1 M1b M2 M2s M3 M4 M5
    A01 A02 A03 A04 A05 A06 A08 A09 A10 A11 D01 M6 M7 M8 M9)
foreach(_tm_case IN LISTS _tm_cases)
    add_test(NAME test_model_${_tm_case}
        COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/test_test_model.py"
        --case "${_tm_case}" --hooks "$<TARGET_FILE:ucr_test_model_hooks>"
        --real "$<TARGET_FILE:ucr_test_model>"
        --limits "$<TARGET_FILE:test_test_model_limits>"
        --viewlib "$<TARGET_FILE:test_model_view>"
        --dnn "$<TARGET_FILE:test_model_view_dnn>"
        --tests "${_tm_tests}"
        --source "${CMAKE_CURRENT_LIST_DIR}" --cmake "${CMAKE_COMMAND}"
        --cxx "${CMAKE_CXX_COMPILER}" --nm "${CMAKE_NM}"
        --revision "${_tm_revision}" --slack "${_tm_slack}")
    set(_tm_labels "threads")
    set(_tm_timeout 120)
    if(_tm_case MATCHES "^M")
        set(_tm_labels "slow")
        set(_tm_timeout 300)
    endif()
    set_tests_properties(test_model_${_tm_case} PROPERTIES LABELS "${_tm_labels}" TIMEOUT ${_tm_timeout})
endforeach()
