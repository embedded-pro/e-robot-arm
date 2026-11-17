# The virtual arm starts processes and opens sockets with POSIX calls: Linux first (virtual-arm.md Part G).
if (UNIX)
    add_executable(e_robot_arm.integration_tests.virtual)
    emil_build_for(e_robot_arm.integration_tests.virtual HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)

    target_include_directories(e_robot_arm.integration_tests.virtual PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    )

    target_compile_definitions(e_robot_arm.integration_tests.virtual PRIVATE
        VIRTUAL_ARM_PROFILE="${CMAKE_CURRENT_LIST_DIR}/virtual-arm.json"
    )

    target_link_libraries(e_robot_arm.integration_tests.virtual PRIVATE
        e_robot_arm.tool.arm_host
        e_robot_arm.tool.arm_host.adapters
        e_robot_arm.tool.tuning
        e_robot_arm.tool.virtual_arm
        cucumber_cpp.runner
        GTest::gtest
    )

    target_sources(e_robot_arm.integration_tests.virtual PRIVATE
        steps/VirtualArmSteps.cpp
        support/VirtualArmFixture.cpp
        support/VirtualArmFixture.hpp
    )
endif()
