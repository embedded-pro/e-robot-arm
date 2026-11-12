add_library(e_robot_arm.integration_tests.support OBJECT
    AccumulatorFixture.cpp
    AccumulatorFixture.hpp
)

target_include_directories(e_robot_arm.integration_tests.support PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.integration_tests.support PUBLIC
    e_robot_arm.example.implementations
    GTest::gtest
    GTest::gmock
)
