add_executable(embedded_scaffold.blinky_cli.main)

target_include_directories(embedded_scaffold.blinky_cli.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.blinky_cli.main PUBLIC
    embedded_scaffold.platform_impl
    embedded_scaffold.blinky_cli
)

target_sources(embedded_scaffold.blinky_cli.main PRIVATE
    Main.cpp
)

# Embedded targets need startup code, a linker script, and firmware artifacts.
# The host build links as a normal executable.
if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(embedded_scaffold.blinky_cli.main)
    halst_target_bringup(embedded_scaffold.blinky_cli.main)
    emil_generate_artifacts(TARGET embedded_scaffold.blinky_cli.main LST MAP BIN HEX)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(embedded_scaffold.blinky_cli.main)
    hal_ti_target_default_init(embedded_scaffold.blinky_cli.main)
    emil_generate_artifacts(TARGET embedded_scaffold.blinky_cli.main LST MAP BIN HEX)
endif()
