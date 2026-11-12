add_executable(e_robot_arm.blinky_cli_test)
emil_build_for(e_robot_arm.blinky_cli_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.blinky_cli_test)

target_link_libraries(e_robot_arm.blinky_cli_test PUBLIC
    gmock_main
    e_robot_arm.blinky_cli
    e_robot_arm.platform_abstraction_mock
    hal.interfaces_test_doubles
    infra.event_test_helper
    infra.timer_test_helper
)

target_sources(e_robot_arm.blinky_cli_test PRIVATE
    TestBlinkyCli.cpp
)
