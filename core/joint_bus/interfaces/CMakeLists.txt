add_library(e_robot_arm.joint_bus.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.joint_bus.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.joint_bus.interfaces INTERFACE
    e_foc_protocol.client
    infra.util
    numerical.math
)

target_sources(e_robot_arm.joint_bus.interfaces PRIVATE
    JointBus.hpp
    JointTransmission.hpp
)
