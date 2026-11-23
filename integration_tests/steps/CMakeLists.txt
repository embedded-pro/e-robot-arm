add_library(e_robot_arm.integration_tests.steps OBJECT
    ArmControllerSteps.cpp
    ArmSteps.cpp
    ModelSteps.cpp
    MotionSteps.cpp
    ProfileSteps.cpp
    SupervisionSteps.cpp
)

target_link_libraries(e_robot_arm.integration_tests.steps PUBLIC
    e_robot_arm.integration_tests.support
    e_robot_arm.arm_profile
    cucumber_cpp.runner
    GTest::gtest
    GTest::gmock
)
