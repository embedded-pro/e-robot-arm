add_library(e_robot_arm.integration_tests.steps OBJECT
    AccumulatorSteps.cpp
)

target_link_libraries(e_robot_arm.integration_tests.steps PUBLIC
    e_robot_arm.integration_tests.support
    cucumber_cpp.runner
    GTest::gtest
    GTest::gmock
)
