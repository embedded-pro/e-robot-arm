add_executable(e_robot_arm.tool.arm_console_test)
emil_build_for(e_robot_arm.tool.arm_console_test HOST All BOOL E_ROBOT_ARM_BUILD_TESTS)
emil_add_test(e_robot_arm.tool.arm_console_test)

target_link_libraries(e_robot_arm.tool.arm_console_test PUBLIC
    gmock_main
    e_robot_arm.tool.arm_console.logic
    e_robot_arm.tool.arm_host.test_doubles
)

target_sources(e_robot_arm.tool.arm_console_test PRIVATE
    TestArmConsole.cpp
    TestCommandTracker.cpp
    TestConsoleOptions.cpp
    TestConsoleSession.cpp
)

if (TARGET e_robot_arm.tool.arm_console AND E_ROBOT_ARM_BUILD_TESTS)
    add_test(NAME e_robot_arm.tool.arm_console.batch_moves
        COMMAND ${CMAKE_COMMAND} -DCONSOLE=$<TARGET_FILE:e_robot_arm.tool.arm_console> -DSCRIPT=${CMAKE_CURRENT_LIST_DIR}/scripts/move.txt
            -DEXPECT_STATUS=0 "-DEXPECT_TEXT=move done" -P ${CMAKE_CURRENT_LIST_DIR}/RunConsoleScript.cmake)
    add_test(NAME e_robot_arm.tool.arm_console.batch_stops_at_a_refusal
        COMMAND ${CMAKE_COMMAND} -DCONSOLE=$<TARGET_FILE:e_robot_arm.tool.arm_console> -DSCRIPT=${CMAKE_CURRENT_LIST_DIR}/scripts/refused.txt
            -DEXPECT_STATUS=1 "-DEXPECT_TEXT=batch stopped at line 3: move 0 0.3 0 0 0 0" -P ${CMAKE_CURRENT_LIST_DIR}/RunConsoleScript.cmake)
endif()
