add_library(embedded_scaffold.platform_abstraction INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.platform_abstraction INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.platform_abstraction INTERFACE
    hal.interfaces
    services.tracer
)

target_sources(embedded_scaffold.platform_abstraction PRIVATE
    Platform.hpp
)

if (SCAFFOLD_BUILD_TESTS)
    add_subdirectory(test_doubles)
endif()
