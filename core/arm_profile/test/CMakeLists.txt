add_executable(e_robot_arm.arm_profile_test)
emil_build_for(e_robot_arm.arm_profile_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.arm_profile_test)

target_link_libraries(e_robot_arm.arm_profile_test PUBLIC
    gmock_main
    e_robot_arm.arm_profile
)

target_sources(e_robot_arm.arm_profile_test PRIVATE
    TestArmProfile.cpp
)
