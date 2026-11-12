add_library(e_robot_arm.platform_host ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.platform_host PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.platform_host PUBLIC
    e_robot_arm.platform_abstraction
    hal.generic
    services.peripheral
    services.tracer
    infra.event
    infra.stream
)

target_sources(e_robot_arm.platform_host PRIVATE
    PlatformImpl.hpp
    PlatformImpl.cpp
)
