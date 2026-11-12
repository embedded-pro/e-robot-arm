add_library(e_robot_arm.joint_bus.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.joint_bus.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.joint_bus.implementations PUBLIC
    e_robot_arm.joint_bus.interfaces
)

target_sources(e_robot_arm.joint_bus.implementations PRIVATE
    JointBusImpl.cpp
    JointBusImpl.hpp
)

add_subdirectory(test)
