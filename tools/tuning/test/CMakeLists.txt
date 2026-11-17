add_executable(e_robot_arm.tool.tuning_test)
emil_build_for(e_robot_arm.tool.tuning_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.tuning_test)

target_link_libraries(e_robot_arm.tool.tuning_test PUBLIC
    gmock_main
    e_robot_arm.tool.tuning
    e_robot_arm.tool.arm_host.test_doubles
)

target_sources(e_robot_arm.tool.tuning_test PRIVATE
    TestTuningMetrics.cpp
    TestTuningRun.cpp
)
