add_executable(e_robot_arm.safety_test)
emil_build_for(e_robot_arm.safety_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.safety_test)

target_link_libraries(e_robot_arm.safety_test PUBLIC
    gmock_main
    e_robot_arm.safety.implementations
    e_robot_arm.joint_bus.test_doubles
)

target_sources(e_robot_arm.safety_test PRIVATE
    TestSafetySupervisorImpl.cpp
)
