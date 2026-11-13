add_library(e_robot_arm.lifecycle.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.lifecycle.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.lifecycle.implementations PUBLIC
    e_robot_arm.joint_configurator.interfaces
    e_robot_arm.lifecycle.interfaces
    e_robot_arm.motion.interfaces
    e_robot_arm.safety.interfaces
    services.fsm
)

target_sources(e_robot_arm.lifecycle.implementations PRIVATE
    ArmLifecycleImpl.cpp
    ArmLifecycleImpl.hpp
)

add_subdirectory(test)
