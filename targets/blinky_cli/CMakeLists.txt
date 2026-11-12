add_executable(e_robot_arm.blinky_cli.main)

target_include_directories(e_robot_arm.blinky_cli.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.blinky_cli.main PUBLIC
    e_robot_arm.platform_impl
    e_robot_arm.blinky_cli
)

target_sources(e_robot_arm.blinky_cli.main PRIVATE
    Main.cpp
)

# Embedded targets need startup code, a linker script, and firmware artifacts.
# The host build links as a normal executable.
if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(e_robot_arm.blinky_cli.main)
    halst_target_bringup(e_robot_arm.blinky_cli.main)
    emil_generate_artifacts(TARGET e_robot_arm.blinky_cli.main LST MAP BIN HEX)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(e_robot_arm.blinky_cli.main)
    hal_ti_target_default_init(e_robot_arm.blinky_cli.main)
    emil_generate_artifacts(TARGET e_robot_arm.blinky_cli.main LST MAP BIN HEX)
endif()
