add_library(e_robot_arm.tool.simulator.view_models STATIC)

target_include_directories(e_robot_arm.tool.simulator.view_models PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.tool.simulator.view_models PUBLIC
    e_robot_arm.tool.arm_host
    e_robot_arm.tool.tuning
    ui.charts
    ui.model
    ui.shell
    ui.stage
    ui.theme
)

target_sources(e_robot_arm.tool.simulator.view_models PRIVATE
    AdapterChoices.cpp
    AdapterChoices.hpp
    ArmControllersPresenter.cpp
    ArmControllersPresenter.hpp
    ArmSceneGeometry.cpp
    ArmSceneGeometry.hpp
    ArmSceneModel.cpp
    ArmSceneModel.hpp
    ConfigurationFlow.cpp
    ConfigurationFlow.hpp
    ControlRules.cpp
    DesktopArm.cpp
    DesktopArm.hpp
    DisturbancePresenter.cpp
    DisturbancePresenter.hpp
    ControlRules.hpp
    EventLog.cpp
    EventLog.hpp
    FrameStatistics.cpp
    FrameStatistics.hpp
    JointRows.cpp
    JointRows.hpp
    JogPresenter.cpp
    JogPresenter.hpp
    MotorControllerPresenter.cpp
    MotorControllerPresenter.hpp
    MotorPresenter.cpp
    MotorPresenter.hpp
    ProfileView.cpp
    ProfileView.hpp
    RecentProfiles.cpp
    RecentProfiles.hpp
    SimulatorPresenter.cpp
    SimulatorPresenter.hpp
    TeachPresenter.cpp
    TeachPresenter.hpp
    ToolReadout.cpp
    TuningPresenter.cpp
    TuningPresenter.hpp
    ToolReadout.hpp
    VirtualMotorRows.cpp
    VirtualMotorRows.hpp
)

add_subdirectory(test)
