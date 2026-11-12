add_executable(e_robot_arm.control_loop_test)
emil_build_for(e_robot_arm.control_loop_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.control_loop_test)

target_link_libraries(e_robot_arm.control_loop_test PUBLIC
    gmock_main
    e_robot_arm.control_loop.implementations
    e_robot_arm.joint_bus.test_doubles
    infra.timer_test_helper
)

target_sources(e_robot_arm.control_loop_test PRIVATE
    TestControlLoopImpl.cpp
)
