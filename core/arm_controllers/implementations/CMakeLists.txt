add_library(e_robot_arm.arm_controllers.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.arm_controllers.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.arm_controllers.implementations PUBLIC
    e_robot_arm.arm_controllers.interfaces
    e_robot_arm.arm_model
    robotics.controllers.manipulator
    e_robot_arm.motion.interfaces
    infra.timer
)

target_sources(e_robot_arm.arm_controllers.implementations PRIVATE
    ArmControllersImpl.cpp
    ArmControllersImpl.hpp
    ComputedTorque.cpp
    ComputedTorque.hpp
    GravityCompensatedPd.cpp
    GravityCompensatedPd.hpp
)

add_subdirectory(test)
