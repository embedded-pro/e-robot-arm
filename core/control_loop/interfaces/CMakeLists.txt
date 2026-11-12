add_library(e_robot_arm.control_loop.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.control_loop.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_sources(e_robot_arm.control_loop.interfaces PRIVATE
    ControlLoop.hpp
)
