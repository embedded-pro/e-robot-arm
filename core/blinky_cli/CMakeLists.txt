add_library(e_robot_arm.blinky_cli ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.blinky_cli PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.blinky_cli PUBLIC
    e_robot_arm.platform_abstraction
    services.util
    services.peripheral
    services.tracer
    infra.timer
    infra.util
)

target_sources(e_robot_arm.blinky_cli PRIVATE
    BlinkyCli.hpp
    BlinkyCli.cpp
)

add_subdirectory(test)
