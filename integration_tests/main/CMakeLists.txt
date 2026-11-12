add_executable(e_robot_arm.integration_tests)
emil_build_for(e_robot_arm.integration_tests HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)

target_link_libraries(e_robot_arm.integration_tests PRIVATE
    e_robot_arm.integration_tests.steps
    e_robot_arm.integration_tests.hooks
    e_robot_arm.integration_tests.support
    cucumber_cpp.runner
)

add_test(NAME e_robot_arm.integration_tests
    COMMAND e_robot_arm.integration_tests -- ${CMAKE_CURRENT_SOURCE_DIR}/../features)
set_tests_properties(e_robot_arm.integration_tests PROPERTIES LABELS integration)
