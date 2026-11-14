add_executable(e_robot_arm.arm_controllers_test)
emil_build_for(e_robot_arm.arm_controllers_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.arm_controllers_test)

target_link_libraries(e_robot_arm.arm_controllers_test PUBLIC
    gmock_main
    e_robot_arm.arm_controllers.implementations
    e_robot_arm.arm_controllers.test_doubles
    e_robot_arm.joint_bus.test_doubles
    e_robot_arm.motion.test_doubles
)

target_sources(e_robot_arm.arm_controllers_test PRIVATE
    TestArmControllersImpl.cpp
    TestJointPositionPassThrough.cpp
    TestSmoothedJointPosition.cpp
)
