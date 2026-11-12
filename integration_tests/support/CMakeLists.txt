add_library(embedded_scaffold.integration_tests.support OBJECT
    AccumulatorFixture.cpp
    AccumulatorFixture.hpp
)

target_include_directories(embedded_scaffold.integration_tests.support PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(embedded_scaffold.integration_tests.support PUBLIC
    embedded_scaffold.example.implementations
    GTest::gtest
    GTest::gmock
)
