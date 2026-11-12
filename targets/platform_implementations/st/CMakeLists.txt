# STM32 platform implementation (STM32F429I-DISC1). Selected by the ST embedded
# presets in CMakePresets.json.
add_library(e_robot_arm.platform_st ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.platform_st PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.platform_st PUBLIC
    e_robot_arm.platform_abstraction
    hal_st.instantiations
    services.tracer
)

# Select the board-specific default clock configuration. Each supported ST board
# preset (see CMakePresets.json) sets TARGET_MCU, which maps to the matching
# hal_st clock header/init function.
set(E_ROBOT_ARM_ST_SUPPORTED_MCUS stm32f407 stm32f429 stm32h563)
if(NOT TARGET_MCU IN_LIST E_ROBOT_ARM_ST_SUPPORTED_MCUS)
    message(FATAL_ERROR "Unsupported TARGET_MCU '${TARGET_MCU}' for e_robot_arm.platform_st. "
        "Supported values: ${E_ROBOT_ARM_ST_SUPPORTED_MCUS}. Add a case in "
        "targets/platform_implementations/st/CMakeLists.txt for new boards.")
endif()

target_compile_definitions(e_robot_arm.platform_st PUBLIC
    $<$<STREQUAL:${TARGET_MCU},stm32f407>:E_ROBOT_ARM_ST_CLOCK_HEADER="hal_st/stm32fxxx/DefaultClockDiscoveryF407G.hpp">
    $<$<STREQUAL:${TARGET_MCU},stm32f407>:E_ROBOT_ARM_ST_CLOCK_INIT=ConfigureDefaultClockDiscoveryF407G>
    $<$<STREQUAL:${TARGET_MCU},stm32f429>:E_ROBOT_ARM_ST_CLOCK_HEADER="hal_st/stm32fxxx/DefaultClockNucleoF429ZI.hpp">
    $<$<STREQUAL:${TARGET_MCU},stm32f429>:E_ROBOT_ARM_ST_CLOCK_INIT=ConfigureDefaultClockNucleoF429ZI>
    $<$<STREQUAL:${TARGET_MCU},stm32h563>:E_ROBOT_ARM_ST_CLOCK_HEADER="hal_st/stm32fxxx/DefaultClockNucleoH563ZI.hpp">
    $<$<STREQUAL:${TARGET_MCU},stm32h563>:E_ROBOT_ARM_ST_CLOCK_INIT=ConfigureDefaultClockNucleoH563ZI>
)

target_sources(e_robot_arm.platform_st PRIVATE
    PlatformImpl.hpp
    PlatformImpl.cpp
)
