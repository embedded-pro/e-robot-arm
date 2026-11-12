add_executable(e_robot_arm.tool.simulator_window_test)
emil_build_for(e_robot_arm.tool.simulator_window_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulator_window_test)
set_tests_properties(e_robot_arm.tool.simulator_window_test PROPERTIES LABELS display)

target_link_libraries(e_robot_arm.tool.simulator_window_test PUBLIC
    gmock
    e_robot_arm.tool.simulator.app
    e_robot_arm.tool.simulator.qml_plugin
)

target_sources(e_robot_arm.tool.simulator_window_test PRIVATE
    TestSimulatorWindow.cpp
)
