file(GLOB ST_L0_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/STM_VL53L0X/core/src/*.c")
file(GLOB ST_L1_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/STM_VL53L1X/core/src/*.c")

add_library(tof_backend STATIC
    LinuxHardware.cpp
    SensorSetup.cpp
    SensorReader.cpp
    StTransport.cpp
    StPlatformL0.cpp
    StPlatformL1.cpp
    ${ST_L0_SOURCES}
    ${ST_L1_SOURCES}
)

target_include_directories(tof_backend
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
    PRIVATE
        vendor/STM_VL53L0X/core/inc
        vendor/STM_VL53L0X/platform/inc
        vendor/STM_VL53L1X/core/inc
        vendor/STM_VL53L1X/platform/inc
)
target_compile_definitions(tof_backend PRIVATE USE_I2C_2V8)
target_compile_options(tof_backend PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>")
target_link_libraries(tof_backend PRIVATE Threads::Threads)
