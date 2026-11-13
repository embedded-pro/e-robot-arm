# Application entry points and platform-specific implementations.
#
# E_ROBOT_ARM_TARGET_PLATFORM selects the active platform (see CMakePresets.json):
#   host preset -> "host"   embedded presets -> "st" / "ti"
# Only the selected platform's implementation is compiled.
add_subdirectory(platform_implementations/${E_ROBOT_ARM_TARGET_PLATFORM})

# Bind the active platform implementation behind the platform::Platform interface.
# A target's Main.cpp includes PLATFORM_IMPL_HEADER to construct the right
# PlatformImpl, while all application logic in core/ depends ONLY on the abstract
# interface (core/platform_abstraction/Platform.hpp). This is the seam that lets the
# same application run on the host and on every supported board.
add_library(e_robot_arm.platform_impl INTERFACE)

target_compile_definitions(e_robot_arm.platform_impl INTERFACE
    PLATFORM_IMPL_HEADER="targets/platform_implementations/${E_ROBOT_ARM_TARGET_PLATFORM}/PlatformImpl.hpp"
)

target_link_libraries(e_robot_arm.platform_impl INTERFACE
    e_robot_arm.platform_${E_ROBOT_ARM_TARGET_PLATFORM}
)
