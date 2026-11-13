add_executable(e_robot_arm.tool.arm_host_test)
emil_build_for(e_robot_arm.tool.arm_host_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.arm_host_test)

target_link_libraries(e_robot_arm.tool.arm_host_test PUBLIC
    gmock_main
    e_robot_arm.tool.arm_host
    infra.timer_test_helper
)

target_sources(e_robot_arm.tool.arm_host_test PRIVATE
    ArmHostTestSupport.hpp
    TestArmMailbox.cpp
    TestArmRequestQueue.cpp
    TestArmRuntime.cpp
    TestArmSession.cpp
    TestProfileFile.cpp
)

add_executable(e_robot_arm.tool.arm_host_thread_test)
emil_build_for(e_robot_arm.tool.arm_host_thread_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.arm_host_thread_test)

target_link_libraries(e_robot_arm.tool.arm_host_thread_test PUBLIC
    gmock_main
    e_robot_arm.tool.arm_host
)

target_sources(e_robot_arm.tool.arm_host_thread_test PRIVATE
    ArmHostTestSupport.hpp
    TestArmHost.cpp
)

if (TARGET e_robot_arm.tool.arm_host.adapters)
    add_executable(e_robot_arm.tool.arm_host_adapters_test)
    emil_build_for(e_robot_arm.tool.arm_host_adapters_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
    emil_add_test(e_robot_arm.tool.arm_host_adapters_test)

    target_link_libraries(e_robot_arm.tool.arm_host_adapters_test PUBLIC
        gmock_main
        e_robot_arm.tool.arm_host.adapters
        infra.timer_test_helper
    )

    target_sources(e_robot_arm.tool.arm_host_adapters_test PRIVATE
        ArmHostTestSupport.hpp
        TestHostTargetOpener.cpp
    )
endif()
