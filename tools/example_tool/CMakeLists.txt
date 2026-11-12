add_executable(embedded_scaffold.tool.example)

target_include_directories(embedded_scaffold.tool.example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.tool.example PUBLIC
    infra.util
    embedded_scaffold.example.implementations
)

target_sources(embedded_scaffold.tool.example PRIVATE
    Main.cpp
)
