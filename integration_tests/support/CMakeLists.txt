add_library(e_robot_arm.integration_tests.support OBJECT
    ArmFixture.cpp
    ArmFixture.hpp
)

target_include_directories(e_robot_arm.integration_tests.support PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.integration_tests.support PUBLIC
    e_robot_arm.arm
    e_foc_protocol.client
    e_robot_arm.tool.simulated_arm
    infra.timer_test_helper
    GTest::gtest
    GTest::gmock
)
