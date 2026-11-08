add_executable(test_t79_acceptance ${CMAKE_CURRENT_LIST_DIR}/test_t79_acceptance.cpp)
target_link_libraries(test_t79_acceptance PRIVATE ucr_control)
target_compile_options(test_t79_acceptance PRIVATE -Wall -Wextra)

foreach(ca A01 A02 A03 A04 A05 A06 A07 A08 A09 A10 A11 A11b A12 A13 A14
           A15 A16 A17 A18 A19 A20 A21 A22 A23 A24 A25 A26 A27 A28 A29 A30 A31)
    add_test(NAME acc_t79_${ca} COMMAND test_t79_acceptance ${ca})
    set_tests_properties(acc_t79_${ca} PROPERTIES LABELS "control;acceptance" TIMEOUT 60)
endforeach()
