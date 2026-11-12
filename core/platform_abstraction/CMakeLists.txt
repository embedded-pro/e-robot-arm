add_library(e_robot_arm.platform_abstraction INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.platform_abstraction INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.platform_abstraction INTERFACE
    hal.interfaces
    services.tracer
)

target_sources(e_robot_arm.platform_abstraction PRIVATE
    Platform.hpp
)

if (E_ROBOT_ARM_BUILD_TESTS)
    add_subdirectory(test_doubles)
endif()
