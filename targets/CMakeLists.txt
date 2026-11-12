# Application entry points and platform-specific implementations.
#
# SCAFFOLD_TARGET_PLATFORM selects the active platform (see CMakePresets.json):
#   host preset -> "host"   embedded presets -> "st" / "ti"
# Only the selected platform's implementation is compiled.
add_subdirectory(platform_implementations/${SCAFFOLD_TARGET_PLATFORM})

# Bind the active platform implementation behind the platform::Platform interface.
# A target's Main.cpp includes PLATFORM_IMPL_HEADER to construct the right
# PlatformImpl, while all application logic in core/ depends ONLY on the abstract
# interface (core/platform_abstraction/Platform.hpp). This is the seam that lets the
# same application run on the host and on every supported board.
add_library(embedded_scaffold.platform_impl INTERFACE)

target_compile_definitions(embedded_scaffold.platform_impl INTERFACE
    PLATFORM_IMPL_HEADER="targets/platform_implementations/${SCAFFOLD_TARGET_PLATFORM}/PlatformImpl.hpp"
)

target_link_libraries(embedded_scaffold.platform_impl INTERFACE
    embedded_scaffold.platform_${SCAFFOLD_TARGET_PLATFORM}
)

# example_app is a trivial host-only demo of using a core library directly.
if (SCAFFOLD_TARGET_PLATFORM STREQUAL "host")
    add_subdirectory(example_app)
endif()

# blinky_cli is the headline application: LED blink + UART command-line interface,
# written once against platform::Platform and built for host, ST and TI.
add_subdirectory(blinky_cli)
