find_package(Qt6 6.4 REQUIRED COMPONENTS Gui Qml Quick QuickWidgets Quick3D Widgets)

add_library(e_robot_arm.tool.simulator.app STATIC)
set_target_properties(e_robot_arm.tool.simulator.app PROPERTIES AUTOMOC ON)

qt_add_qml_module(e_robot_arm.tool.simulator.app
    URI Simulator
    VERSION 1.0
    PLUGIN_TARGET e_robot_arm.tool.simulator.qml_plugin
    CLASS_NAME SimulatorPlugin
    OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Simulator
    DEPENDENCIES QtQuick QtQuick3D
    NO_CACHEGEN
    QML_FILES
        qml/ArmModel.qml
        qml/ArmPart.qml
        qml/ArmScene.qml
    RESOURCES
        assets/studio.hdr
    SOURCES
        ArmSceneState.cpp
        ArmSceneState.hpp
        TrailGeometry.cpp
        TrailGeometry.hpp
)

target_include_directories(e_robot_arm.tool.simulator.app PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_sources(e_robot_arm.tool.simulator.app PRIVATE
    EmergencyStopKey.cpp
    EmergencyStopKey.hpp
    JointTable.cpp
    JointTable.hpp
    Panels.cpp
    Panels.hpp
    SimulatorWindow.cpp
    SimulatorWindow.hpp
    StatusStripBar.cpp
    StatusStripBar.hpp
)

target_link_libraries(e_robot_arm.tool.simulator.app PUBLIC
    e_robot_arm.tool.simulator.view_models
    e_robot_arm.tool.arm_host.adapters
    ui.backend.qt
    ui.scope
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::QuickWidgets
    Qt6::Quick3D
    Qt6::Widgets
)

add_executable(e_robot_arm.tool.simulator)
set_target_properties(e_robot_arm.tool.simulator PROPERTIES OUTPUT_NAME e-robot-arm-simulator)

target_sources(e_robot_arm.tool.simulator PRIVATE
    main/Main.cpp
)

target_link_libraries(e_robot_arm.tool.simulator PRIVATE
    e_robot_arm.tool.simulator.qml_plugin
    e_robot_arm.tool.simulator.app
)
add_subdirectory(test)
