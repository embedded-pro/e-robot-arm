add_library(e_robot_arm.tool.arm_host STATIC)

target_include_directories(e_robot_arm.tool.arm_host PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.arm_host PUBLIC
    e_robot_arm.arm
    e_robot_arm.arm_profile
    e_robot_arm.tool.simulated_arm
    e_robot_arm.tool.virtual_arm
    e_foc_protocol.client
    hal.generic
    infra.event
    infra.timer
)

target_sources(e_robot_arm.tool.arm_host PRIVATE
    ArmCommand.cpp
    ArmCommand.hpp
    ArmCommands.hpp
    ArmEvent.hpp
    ArmHost.cpp
    ArmHost.hpp
    ArmMailbox.cpp
    ArmMailbox.hpp
    ArmRuntime.cpp
    ArmRuntime.hpp
    ArmSession.cpp
    ArmSession.hpp
    Commissioning.cpp
    Commissioning.hpp
    ArmSnapshot.hpp
    ArmTarget.hpp
    ArmTargetLink.cpp
    ArmTargetLink.hpp
    DefaultArmConfig.cpp
    DefaultArmConfig.hpp
    ProfileFile.cpp
    ProfileFile.hpp
    VirtualMotors.hpp
)

if (TARGET can_lite.drivers.platform)
    add_library(e_robot_arm.tool.arm_host.adapters STATIC)

    target_link_libraries(e_robot_arm.tool.arm_host.adapters PUBLIC
        e_robot_arm.tool.arm_host
        can_lite.drivers.hub
        can_lite.drivers.platform
    )

    target_sources(e_robot_arm.tool.arm_host.adapters PRIVATE
        AdapterTargetLink.cpp
        AdapterTargetLink.hpp
    )

    # The virtual arm starts its motors as processes with POSIX calls: Linux first (virtual-arm.md).
    if (UNIX)
        add_library(e_robot_arm.tool.arm_host.virtual_arm STATIC)

        target_link_libraries(e_robot_arm.tool.arm_host.virtual_arm PUBLIC
            e_robot_arm.tool.arm_host.adapters
            e_robot_arm.tool.virtual_arm
        )

        target_sources(e_robot_arm.tool.arm_host.virtual_arm PRIVATE
            VirtualArmTargetLink.cpp
            VirtualArmTargetLink.hpp
        )
    endif()
endif()

add_library(e_robot_arm.tool.arm_host.test_doubles INTERFACE)

target_link_libraries(e_robot_arm.tool.arm_host.test_doubles INTERFACE
    e_robot_arm.tool.arm_host
    gmock
)

add_subdirectory(test)
