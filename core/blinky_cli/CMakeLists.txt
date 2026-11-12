add_library(embedded_scaffold.blinky_cli ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.blinky_cli PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.blinky_cli PUBLIC
    embedded_scaffold.platform_abstraction
    services.util
    services.peripheral
    services.tracer
    infra.timer
    infra.util
)

target_sources(embedded_scaffold.blinky_cli PRIVATE
    BlinkyCli.hpp
    BlinkyCli.cpp
)

add_subdirectory(test)
