add_executable(e_robot_arm.tool.virtual_arm_test)
emil_build_for(e_robot_arm.tool.virtual_arm_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.virtual_arm_test)

target_link_libraries(e_robot_arm.tool.virtual_arm_test PUBLIC
    gmock_main
    e_robot_arm.tool.virtual_arm
)

target_sources(e_robot_arm.tool.virtual_arm_test PRIVATE
    TestArmCoupling.cpp
    TestVirtualArm.cpp
    TestVirtualArmPlan.cpp
)

add_executable(e_robot_arm.tool.virtual_arm.fake_virtual_motor)
emil_build_for(e_robot_arm.tool.virtual_arm.fake_virtual_motor HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)

target_link_libraries(e_robot_arm.tool.virtual_arm.fake_virtual_motor PRIVATE
    e_robot_arm.tool.virtual_arm.plant_port
)

target_sources(e_robot_arm.tool.virtual_arm.fake_virtual_motor PRIVATE
    FakeVirtualMotor.cpp
)

if (TARGET e_robot_arm.tool.virtual_arm_test)
    add_dependencies(e_robot_arm.tool.virtual_arm_test e_robot_arm.tool.virtual_arm.fake_virtual_motor)
    target_compile_definitions(e_robot_arm.tool.virtual_arm_test PRIVATE FAKE_VIRTUAL_MOTOR="$<TARGET_FILE:e_robot_arm.tool.virtual_arm.fake_virtual_motor>")
endif()
