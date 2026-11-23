add_executable(e_robot_arm.tool.simulator.samples_test)
emil_build_for(e_robot_arm.tool.simulator.samples_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulator.samples_test)

target_link_libraries(e_robot_arm.tool.simulator.samples_test PUBLIC
    gmock_main
    e_robot_arm.tool.simulator.samples
)

target_sources(e_robot_arm.tool.simulator.samples_test PRIVATE
    TestSampleArm.cpp
)

# The committed files hold float bits from one C library's sin, cos and pow; compare them only where they were made.
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(e_robot_arm.tool.simulator.samples_test PRIVATE SAMPLE_MESHES_DIR="${CMAKE_CURRENT_LIST_DIR}/../meshed-arm")
endif()
