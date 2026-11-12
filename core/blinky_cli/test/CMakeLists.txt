add_executable(embedded_scaffold.blinky_cli_test)
emil_build_for(embedded_scaffold.blinky_cli_test HOST All BOOL SCAFFOLD_BUILD_TESTS)
emil_add_test(embedded_scaffold.blinky_cli_test)

target_link_libraries(embedded_scaffold.blinky_cli_test PUBLIC
    gmock_main
    embedded_scaffold.blinky_cli
    embedded_scaffold.platform_abstraction_mock
    hal.interfaces_test_doubles
    infra.event_test_helper
    infra.timer_test_helper
)

target_sources(embedded_scaffold.blinky_cli_test PRIVATE
    TestBlinkyCli.cpp
)
