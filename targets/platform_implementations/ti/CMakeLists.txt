# TI (Tiva C) platform implementation (EK-TM4C123GXL). Selected by the TI embedded
# presets in CMakePresets.json.
add_library(embedded_scaffold.platform_ti ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.platform_ti PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.platform_ti PUBLIC
    embedded_scaffold.platform_abstraction
    hal_tiva.instantiations
    hal_tiva.tiva
    services.tracer
)

target_sources(embedded_scaffold.platform_ti PRIVATE
    PlatformImpl.hpp
    PlatformImpl.cpp
)
