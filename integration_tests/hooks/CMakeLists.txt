add_library(embedded_scaffold.integration_tests.hooks OBJECT
    Hooks.cpp
)

target_link_libraries(embedded_scaffold.integration_tests.hooks PUBLIC
    cucumber_cpp.runner
)
