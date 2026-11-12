add_library(e_robot_arm.tool.arm_console.logic STATIC)

target_include_directories(e_robot_arm.tool.arm_console.logic PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.arm_console.logic PUBLIC
    e_robot_arm.tool.arm_host
)

target_sources(e_robot_arm.tool.arm_console.logic PRIVATE
    ArmConsole.cpp
    ArmConsole.hpp
    ConsoleOptions.cpp
    ConsoleOptions.hpp
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
endif()

add_subdirectory(test)
