add_executable(e_robot_arm.joint_configurator_test)
emil_build_for(e_robot_arm.joint_configurator_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.joint_configurator_test)

target_link_libraries(e_robot_arm.joint_configurator_test PUBLIC
    gmock_main
    e_robot_arm.joint_configurator.implementations
    e_robot_arm.joint_configurator.test_doubles
    e_robot_arm.joint_bus.test_doubles
    e_robot_arm.lifecycle.test_doubles
    infra.timer_test_helper
)

target_sources(e_robot_arm.joint_configurator_test PRIVATE
    TestControllerSettings.cpp
    TestJointConfiguratorImpl.cpp
)
