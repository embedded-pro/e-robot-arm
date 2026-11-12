add_executable(e_robot_arm.tool.simulated_arm_test)
emil_build_for(e_robot_arm.tool.simulated_arm_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulated_arm_test)

target_link_libraries(e_robot_arm.tool.simulated_arm_test PUBLIC
    gmock_main
    e_robot_arm.tool.simulated_arm
)

target_sources(e_robot_arm.tool.simulated_arm_test PRIVATE
    TestFirstOrderLag.cpp
)
