add_executable(e_robot_arm.joint_bus_test)
emil_build_for(e_robot_arm.joint_bus_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.joint_bus_test)

target_link_libraries(e_robot_arm.joint_bus_test PUBLIC
    gmock_main
    e_robot_arm.joint_bus.implementations
    e_robot_arm.joint_bus.test_doubles
    can_lite.test_util
    infra.timer_test_helper
)

target_sources(e_robot_arm.joint_bus_test PRIVATE
    TestJointBusImpl.cpp
    TestJointTransmission.cpp
)
