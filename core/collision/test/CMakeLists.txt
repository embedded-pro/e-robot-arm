add_executable(e_robot_arm.collision_test)
emil_build_for(e_robot_arm.collision_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.collision_test)

target_link_libraries(e_robot_arm.collision_test PUBLIC
    gmock_main
    e_robot_arm.collision
)

target_sources(e_robot_arm.collision_test PRIVATE
    TestCollisionObserver.cpp
)
