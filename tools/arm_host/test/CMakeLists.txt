add_executable(e_robot_arm.tool.arm_host_test)
emil_build_for(e_robot_arm.tool.arm_host_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.arm_host_test)

target_link_libraries(e_robot_arm.tool.arm_host_test PUBLIC
    gmock_main
    e_robot_arm.tool.arm_host
    infra.timer_test_helper
    hal.interfaces_test_doubles
)

target_sources(e_robot_arm.tool.arm_host_test PRIVATE
    ArmHostTestSupport.hpp
    TestArmMailbox.cpp
    TestArmRequestQueue.cpp
    TestArmRuntime.cpp
    TestArmSession.cpp
    TestBusLoadMeter.cpp
    TestCommissioning.cpp
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
    set_tests_properties(e_robot_arm.tool.arm_host_adapters_test PROPERTIES RESOURCE_LOCK virtual_can_hub_port)

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

if (TARGET e_robot_arm.tool.arm_host.virtual_arm AND TARGET e_robot_arm.tool.virtual_arm.fake_virtual_motor)
    add_executable(e_robot_arm.tool.arm_host_virtual_arm_test)
    emil_build_for(e_robot_arm.tool.arm_host_virtual_arm_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
    emil_add_test(e_robot_arm.tool.arm_host_virtual_arm_test)
    set_tests_properties(e_robot_arm.tool.arm_host_virtual_arm_test PROPERTIES RESOURCE_LOCK virtual_can_hub_port)

    target_link_libraries(e_robot_arm.tool.arm_host_virtual_arm_test PUBLIC
        gmock_main
        e_robot_arm.tool.arm_host.virtual_arm
        infra.timer_test_helper
    )

    target_sources(e_robot_arm.tool.arm_host_virtual_arm_test PRIVATE
        ArmHostTestSupport.hpp
        TestVirtualArmTargetLink.cpp
    )

    add_dependencies(e_robot_arm.tool.arm_host_virtual_arm_test e_robot_arm.tool.virtual_arm.fake_virtual_motor)
    target_compile_definitions(e_robot_arm.tool.arm_host_virtual_arm_test PRIVATE FAKE_VIRTUAL_MOTOR="$<TARGET_FILE:e_robot_arm.tool.virtual_arm.fake_virtual_motor>")
endif()
