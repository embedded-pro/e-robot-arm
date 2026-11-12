# TI (Tiva C) platform implementation (EK-TM4C123GXL). Selected by the TI embedded
# presets in CMakePresets.json.
add_library(e_robot_arm.platform_ti ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.platform_ti PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.platform_ti PUBLIC
    e_robot_arm.platform_abstraction
    hal_tiva.instantiations
    hal_tiva.tiva
    services.tracer
)

target_sources(e_robot_arm.platform_ti PRIVATE
    PlatformImpl.hpp
    PlatformImpl.cpp
)
