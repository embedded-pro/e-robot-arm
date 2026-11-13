add_executable(e_robot_arm.arm_model_test)
emil_build_for(e_robot_arm.arm_model_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.arm_model_test)

target_link_libraries(e_robot_arm.arm_model_test PUBLIC
    gmock_main
    e_robot_arm.arm_model
)

target_sources(e_robot_arm.arm_model_test PRIVATE
    TestArmModel.cpp
)
