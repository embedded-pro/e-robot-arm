add_executable(e_robot_arm.tool.arm_console_test)
emil_build_for(e_robot_arm.tool.arm_console_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.arm_console_test)

target_link_libraries(e_robot_arm.tool.arm_console_test PUBLIC
    gmock_main
    e_robot_arm.tool.arm_console.logic
    e_robot_arm.tool.arm_host.test_doubles
)

target_sources(e_robot_arm.tool.arm_console_test PRIVATE
    TestArmConsole.cpp
    TestConsoleOptions.cpp
)
