add_library(e_robot_arm.integration_tests.support OBJECT
    AccumulatorFixture.cpp
    AccumulatorFixture.hpp
    ArmFixture.cpp
    ArmFixture.hpp
    SimulatedJoint.cpp
    SimulatedJoint.hpp
    VirtualCanBus.cpp
    VirtualCanBus.hpp
)

target_include_directories(e_robot_arm.integration_tests.support PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.integration_tests.support PUBLIC
    e_robot_arm.example.implementations
    e_robot_arm.arm
    e_foc_protocol.client
    e_foc_protocol.server
    can_lite.server
    infra.timer_test_helper
    GTest::gtest
    GTest::gmock
)
