#include PLATFORM_IMPL_HEADER
#include "core/blinky_cli/BlinkyCli.hpp"

// Platform-agnostic entry point.
//
// PLATFORM_IMPL_HEADER is set per platform by targets/CMakeLists.txt and resolves to
// the selected board's PlatformImpl (host / st / ti). The application is constructed
// against the abstract platform::Platform interface, then the platform's event loop
// takes over. This single Main.cpp is reused unchanged across every target.
int main()
{
    static application::PlatformImpl platform;
    static application::BlinkyCli blinkyCli{ platform };

    platform.Run();

#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#endif
}
