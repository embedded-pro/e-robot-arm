add_library(e_robot_arm.lifecycle.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.lifecycle.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.lifecycle.interfaces INTERFACE
    e_robot_arm.joint_bus.interfaces
)

target_sources(e_robot_arm.lifecycle.interfaces PRIVATE
    ArmLifecycle.hpp
)
