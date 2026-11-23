add_executable(e_robot_arm.tool.simulator_window_test)
emil_build_for(e_robot_arm.tool.simulator_window_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulator_window_test)
set_tests_properties(e_robot_arm.tool.simulator_window_test PROPERTIES LABELS display)

target_link_libraries(e_robot_arm.tool.simulator_window_test PUBLIC
    gmock
    e_robot_arm.tool.simulator.qml_plugin
    ui.backend.quick3d_plugin
    e_robot_arm.tool.simulator.app
)

if (TARGET e_robot_arm.tool.arm_host.virtual_arm)
    target_link_libraries(e_robot_arm.tool.simulator_window_test PRIVATE e_robot_arm.tool.arm_host.virtual_arm)
    target_compile_definitions(e_robot_arm.tool.simulator_window_test PRIVATE
        E_ROBOT_ARM_VIRTUAL_ARM
        VIRTUAL_ARM_PROFILE="${e_robot_arm_SOURCE_DIR}/integration_tests/virtual/virtual-arm.json"
    )
endif()

target_sources(e_robot_arm.tool.simulator_window_test PRIVATE
    TestSimulatorWindow.cpp
)

target_compile_definitions(e_robot_arm.tool.simulator_window_test PRIVATE
    MESH_TEST_DATA="${CMAKE_CURRENT_LIST_DIR}/data"
    SAMPLE_ARM_PROFILE="${e_robot_arm_SOURCE_DIR}/tools/simulator/samples/meshed-arm/profile.json"
)

add_executable(e_robot_arm.tool.simulator.mesh_files_test)
emil_build_for(e_robot_arm.tool.simulator.mesh_files_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulator.mesh_files_test)

target_link_libraries(e_robot_arm.tool.simulator.mesh_files_test PUBLIC
    gmock_main
    e_robot_arm.tool.simulator.app
)

target_sources(e_robot_arm.tool.simulator.mesh_files_test PRIVATE
    TestMeshFiles.cpp
)

target_compile_definitions(e_robot_arm.tool.simulator.mesh_files_test PRIVATE MESH_TEST_DATA="${CMAKE_CURRENT_LIST_DIR}/data")
