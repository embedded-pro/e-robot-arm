add_library(embedded_scaffold.platform_host ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.platform_host PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.platform_host PUBLIC
    embedded_scaffold.platform_abstraction
    hal.generic
    services.peripheral
    services.tracer
    infra.event
    infra.stream
)

target_sources(embedded_scaffold.platform_host PRIVATE
    PlatformImpl.hpp
    PlatformImpl.cpp
)
