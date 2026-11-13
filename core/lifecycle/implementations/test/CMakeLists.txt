add_executable(e_robot_arm.lifecycle_test)
emil_build_for(e_robot_arm.lifecycle_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.lifecycle_test)

target_link_libraries(e_robot_arm.lifecycle_test PUBLIC
    gmock_main
    e_robot_arm.lifecycle.implementations
    e_robot_arm.joint_configurator.test_doubles
    e_robot_arm.lifecycle.test_doubles
    e_robot_arm.motion.test_doubles
    e_robot_arm.safety.test_doubles
)

target_sources(e_robot_arm.lifecycle_test PRIVATE
    TestArmLifecycleImpl.cpp
)
