add_library(e_robot_arm.arm ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.arm PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.arm PUBLIC
    e_robot_arm.arm_model
    e_robot_arm.control_loop.implementations
    e_robot_arm.joint_bus.implementations
    e_robot_arm.joint_configurator.implementations
    e_robot_arm.lifecycle.implementations
    e_robot_arm.motion.implementations
    e_robot_arm.safety.implementations
)

target_sources(e_robot_arm.arm PRIVATE
    Arm.cpp
    Arm.hpp
    ArmConfig.hpp
)

add_subdirectory(test)
