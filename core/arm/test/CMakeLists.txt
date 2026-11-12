add_executable(e_robot_arm.arm_test)
emil_build_for(e_robot_arm.arm_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.arm_test)

target_link_libraries(e_robot_arm.arm_test PUBLIC
    gmock_main
    e_robot_arm.arm
    can_lite.test_util
    e_foc_protocol.client
    infra.timer_test_helper
)

target_sources(e_robot_arm.arm_test PRIVATE
    TestArm.cpp
)
