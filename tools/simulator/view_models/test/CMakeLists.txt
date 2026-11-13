add_executable(e_robot_arm.tool.simulator.view_models_test)
emil_build_for(e_robot_arm.tool.simulator.view_models_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.simulator.view_models_test)

target_link_libraries(e_robot_arm.tool.simulator.view_models_test PUBLIC
    gmock_main
    e_robot_arm.tool.simulator.view_models
    e_robot_arm.tool.arm_host.test_doubles
    ui.backend.recording
)

target_sources(e_robot_arm.tool.simulator.view_models_test PRIVATE
    SnapshotBuilder.hpp
    TestArmSceneModel.cpp
    TestControlRules.cpp
    TestEventLog.cpp
    TestJointRows.cpp
    TestMotorControllerPresenter.cpp
    TestMotorPresenter.cpp
    TestProfileView.cpp
    TestSimulatorPresenter.cpp
    TestToolReadout.cpp
)
