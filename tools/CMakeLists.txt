# Host-side developer tools (CLI utilities, simulators, bridges).
#
# The top-level CMakeLists only adds this directory for non-embedded builds, so
# everything here targets the host.
add_subdirectory(arm_console)
add_subdirectory(simulated_arm)
add_subdirectory(virtual_arm)
add_subdirectory(arm_host)
add_subdirectory(tuning)

add_subdirectory(simulator)

if (TARGET e_robot_arm.tool.simulator)
    add_subdirectory(installer)
endif()
