add_executable(embedded_scaffold.example_app)

target_include_directories(embedded_scaffold.example_app PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.example_app PUBLIC
    infra.util
    embedded_scaffold.example.implementations
)

target_sources(embedded_scaffold.example_app PRIVATE
    Main.cpp
)
