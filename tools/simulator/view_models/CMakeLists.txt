add_library(e_robot_arm.tool.simulator.view_models STATIC)

target_include_directories(e_robot_arm.tool.simulator.view_models PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.tool.simulator.view_models PUBLIC
    e_robot_arm.tool.arm_host
    ui.model
    ui.shell
    ui.stage
    ui.theme
)

target_sources(e_robot_arm.tool.simulator.view_models PRIVATE
    ArmSceneGeometry.cpp
    ArmSceneGeometry.hpp
    ArmSceneModel.cpp
    ArmSceneModel.hpp
    ConfigurationFlow.cpp
    ConfigurationFlow.hpp
    ControlRules.cpp
    DesktopArm.cpp
    DesktopArm.hpp
    ControlRules.hpp
    EventLog.cpp
    EventLog.hpp
    JointRows.cpp
    JointRows.hpp
    MotorControllerPresenter.cpp
    MotorControllerPresenter.hpp
    MotorPresenter.cpp
    MotorPresenter.hpp
    ProfileView.cpp
    ProfileView.hpp
    SimulatorPresenter.cpp
    SimulatorPresenter.hpp
    ToolReadout.cpp
    ToolReadout.hpp
)

add_subdirectory(test)
