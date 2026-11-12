add_library(e_robot_arm.platform_abstraction_mock INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.platform_abstraction_mock INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.platform_abstraction_mock INTERFACE
    e_robot_arm.platform_abstraction
    gmock
)

target_sources(e_robot_arm.platform_abstraction_mock INTERFACE
    PlatformMock.hpp
)
