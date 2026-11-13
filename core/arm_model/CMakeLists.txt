add_library(e_robot_arm.arm_model ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.arm_model PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.arm_model PUBLIC
    e_robot_arm.joint_bus.interfaces
    robotics.kinematics
    robotics.dynamics
    numerical.math
    numerical.solver
)

target_sources(e_robot_arm.arm_model PRIVATE
    ArmGeometry.hpp
    ArmMass.hpp
    ArmModel.cpp
    ArmModel.hpp
)

add_subdirectory(test)
