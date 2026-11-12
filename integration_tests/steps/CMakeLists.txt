add_library(embedded_scaffold.integration_tests.steps OBJECT
    AccumulatorSteps.cpp
)

target_link_libraries(embedded_scaffold.integration_tests.steps PUBLIC
    embedded_scaffold.integration_tests.support
    cucumber_cpp.runner
    GTest::gtest
    GTest::gmock
)
