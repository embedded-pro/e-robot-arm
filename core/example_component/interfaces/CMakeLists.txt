add_library(embedded_scaffold.example.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.example.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.example.interfaces INTERFACE
    infra.util
)

target_sources(embedded_scaffold.example.interfaces PRIVATE
    Accumulator.hpp
)
