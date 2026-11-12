add_library(e_robot_arm.control_loop.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.control_loop.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.control_loop.implementations PUBLIC
    e_robot_arm.control_loop.interfaces
    e_robot_arm.joint_bus.interfaces
    infra.timer
)

target_sources(e_robot_arm.control_loop.implementations PRIVATE
    ControlLoopImpl.cpp
    ControlLoopImpl.hpp
)

add_subdirectory(test)
