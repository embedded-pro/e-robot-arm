add_library(e_robot_arm.tool.simulated_arm STATIC)

target_include_directories(e_robot_arm.tool.simulated_arm PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.simulated_arm PUBLIC
    e_robot_arm.joint_bus.interfaces
    e_robot_arm.joint_configurator.interfaces
    e_foc_protocol.server
    can_lite.server
    infra.event
    infra.timer
)

target_sources(e_robot_arm.tool.simulated_arm PRIVATE
    FirstOrderLag.cpp
    FirstOrderLag.hpp
    SimulatedArm.cpp
    SimulatedArm.hpp
    SimulatedJoint.cpp
    SimulatedJoint.hpp
    VirtualCanBus.cpp
    VirtualCanBus.hpp
)

add_subdirectory(test)
