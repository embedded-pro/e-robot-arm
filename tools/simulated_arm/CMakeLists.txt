add_library(e_robot_arm.tool.simulated_arm STATIC)

target_include_directories(e_robot_arm.tool.simulated_arm PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.simulated_arm PUBLIC
    e_robot_arm.arm_model
    e_robot_arm.joint_bus.interfaces
    e_robot_arm.joint_configurator.interfaces
    e_robot_arm.safety.interfaces
    e_foc_protocol.server
    can_lite.server
    infra.event
    infra.timer
)

target_sources(e_robot_arm.tool.simulated_arm PRIVATE
    ArmDynamics.cpp
    ArmDynamics.hpp
    ArmPlant.cpp
    ArmPlant.hpp
    FirstOrderLag.cpp
    FirstOrderLag.hpp
    JointPlant.hpp
    SimulatedArm.cpp
    SimulatedArm.hpp
    SimulatedHardwareStop.cpp
    SimulatedHardwareStop.hpp
    SimulatedJoint.cpp
    SimulatedJoint.hpp
    VirtualCanBus.cpp
    VirtualCanBus.hpp
)

add_subdirectory(test)
