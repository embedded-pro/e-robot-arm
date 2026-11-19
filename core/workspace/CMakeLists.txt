add_library(e_robot_arm.workspace ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.workspace PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.workspace PUBLIC
    e_robot_arm.arm_model
    e_robot_arm.safety.interfaces
    robotics.planning
)

target_sources(e_robot_arm.workspace PRIVATE
    WorkspaceCheck.cpp
    WorkspaceCheck.hpp
    WorkspaceConfig.hpp
)

add_subdirectory(test)
