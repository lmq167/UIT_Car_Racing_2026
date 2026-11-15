# Kiểm driver Motor thật qua UART giả (pty), không liên kết app_core.
add_executable(acc_motor_uart acc_motor_uart.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/Motor.cpp
    ${PROJECT_SOURCE_DIR}/src/common/Logger.cpp)
target_include_directories(acc_motor_uart PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(acc_motor_uart PRIVATE Threads::Threads)
add_test(NAME acc_motor_uart COMMAND acc_motor_uart)
set_tests_properties(acc_motor_uart PROPERTIES LABELS "acceptance" TIMEOUT 120)
