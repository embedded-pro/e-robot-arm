add_executable(embedded_scaffold.example_test)
emil_build_for(embedded_scaffold.example_test HOST All BOOL SCAFFOLD_BUILD_TESTS)
emil_add_test(embedded_scaffold.example_test)

target_link_libraries(embedded_scaffold.example_test PUBLIC
    gmock_main
    embedded_scaffold.example.implementations
)

target_sources(embedded_scaffold.example_test PRIVATE
    TestAccumulatorImpl.cpp
)
