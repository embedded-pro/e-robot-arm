add_library(e_robot_arm.tool.simulator.view_models STATIC)

target_include_directories(e_robot_arm.tool.simulator.view_models PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.tool.simulator.view_models PUBLIC
    e_robot_arm.tool.arm_host
    ui.model
    ui.shell
    ui.theme
)

target_sources(e_robot_arm.tool.simulator.view_models PRIVATE
    ArmSceneModel.cpp
    ArmSceneModel.hpp
    ControlRules.cpp
    ControlRules.hpp
    EventLog.cpp
    EventLog.hpp
    JointRows.cpp
    JointRows.hpp
    SimulatorPresenter.cpp
    SimulatorPresenter.hpp
)

add_subdirectory(test)
