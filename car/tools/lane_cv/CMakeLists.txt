cmake_minimum_required(VERSION 3.10)
# Có thể cấu hình riêng thư mục này để không cần các phụ thuộc điều khiển.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(UCRLaneCV LANGUAGES CXX)
endif()
find_package(OpenCV 4.1 REQUIRED COMPONENTS core imgproc imgcodecs)
add_executable(ucr_lane_cv lane_cv.cpp)
set_target_properties(ucr_lane_cv PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_include_directories(ucr_lane_cv PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ucr_lane_cv PRIVATE opencv_core opencv_imgproc opencv_imgcodecs)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(ucr_lane_cv PRIVATE stdc++fs)
endif()
include(CTest)
if(BUILD_TESTING)
    add_test(NAME lane_cv_synthetic COMMAND ${CMAKE_COMMAND}
        -DPROGRAM=$<TARGET_FILE:ucr_lane_cv>
        -DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test-output
        -P ${CMAKE_CURRENT_SOURCE_DIR}/test_cli.cmake)
    set_tests_properties(lane_cv_synthetic PROPERTIES TIMEOUT 60)
    add_test(NAME lane_cv_symbols COMMAND ${CMAKE_COMMAND}
        -DPROGRAM=$<TARGET_FILE:ucr_lane_cv> -DNM=${CMAKE_NM}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_symbols.cmake)
    set_tests_properties(lane_cv_symbols PROPERTIES TIMEOUT 20)
endif()
