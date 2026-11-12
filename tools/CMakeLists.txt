# Host-side developer tools (CLI utilities, simulators, bridges).
#
# The top-level CMakeLists only adds this directory for non-embedded builds, so
# everything here targets the host.
add_subdirectory(example_tool)

# Qt-based GUI tools (simulators, dashboards): provide Qt6 and enable here.
# find_package(Qt6 QUIET COMPONENTS Network Widgets)
# if (Qt6_FOUND)
#     qt_standard_project_setup()
#     add_subdirectory(simulator)
# endif()
