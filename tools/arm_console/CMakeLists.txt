add_library(e_robot_arm.tool.arm_console.logic STATIC)

target_include_directories(e_robot_arm.tool.arm_console.logic PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.arm_console.logic PUBLIC
    e_robot_arm.tool.arm_host
    e_robot_arm.tool.tuning
)

target_sources(e_robot_arm.tool.arm_console.logic PRIVATE
    ArmConsole.cpp
    ArmConsole.hpp
    CommandTracker.cpp
    CommandTracker.hpp
    ConsoleOptions.cpp
    ConsoleOptions.hpp
    ConsoleSession.cpp
    ConsoleSession.hpp
)

if (TARGET can_lite.drivers.platform)
    add_executable(e_robot_arm.tool.arm_console)
    set_target_properties(e_robot_arm.tool.arm_console PROPERTIES OUTPUT_NAME e-robot-arm-console)

    target_link_libraries(e_robot_arm.tool.arm_console PRIVATE
        e_robot_arm.tool.arm_console.logic
        e_robot_arm.tool.arm_host.adapters
    )

    target_sources(e_robot_arm.tool.arm_console PRIVATE
        main/Main.cpp
    )

    if (UNIX)
        target_link_libraries(e_robot_arm.tool.arm_console PRIVATE e_robot_arm.tool.virtual_arm)
        target_compile_definitions(e_robot_arm.tool.arm_console PRIVATE E_ROBOT_ARM_VIRTUAL_ARM)
    endif()
endif()

add_subdirectory(test)
