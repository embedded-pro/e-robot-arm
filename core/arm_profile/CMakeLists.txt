add_library(e_robot_arm.arm_profile ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.arm_profile PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.arm_profile PUBLIC
    e_robot_arm.arm
    e_foc_protocol.contract
    infra.syntax
)

target_sources(e_robot_arm.arm_profile PRIVATE
    ArmProfile.cpp
    ArmProfile.hpp
)

add_subdirectory(test)
