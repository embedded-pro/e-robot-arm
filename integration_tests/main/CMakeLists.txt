add_executable(embedded_scaffold.integration_tests)
emil_build_for(embedded_scaffold.integration_tests HOST All BOOL SCAFFOLD_BUILD_TESTS)

target_link_libraries(embedded_scaffold.integration_tests PRIVATE
    embedded_scaffold.integration_tests.steps
    embedded_scaffold.integration_tests.hooks
    embedded_scaffold.integration_tests.support
    cucumber_cpp.runner
)

add_test(NAME embedded_scaffold.integration_tests
    COMMAND embedded_scaffold.integration_tests -- ${CMAKE_CURRENT_SOURCE_DIR}/../features)
set_tests_properties(embedded_scaffold.integration_tests PROPERTIES LABELS integration)
