add_library(embedded_scaffold.platform_abstraction_mock INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.platform_abstraction_mock INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(embedded_scaffold.platform_abstraction_mock INTERFACE
    embedded_scaffold.platform_abstraction
    gmock
)

target_sources(embedded_scaffold.platform_abstraction_mock INTERFACE
    PlatformMock.hpp
)
