# T28-C thêm include file này trong BUILD_TESTING; không cần OpenCV/phần cứng.
if(NOT BUILD_TESTING)
    return()
endif()
add_executable(test_pass_store
    "${CMAKE_CURRENT_LIST_DIR}/test_pass_store.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/pass_store.cpp")
set_target_properties(test_pass_store PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_pass_store PRIVATE -Wall -Wextra -Werror)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(test_pass_store PRIVATE stdc++fs)
endif()
add_test(NAME drive_collect_pass_store COMMAND test_pass_store)
set_tests_properties(drive_collect_pass_store PROPERTIES LABELS "unit" TIMEOUT 30)
