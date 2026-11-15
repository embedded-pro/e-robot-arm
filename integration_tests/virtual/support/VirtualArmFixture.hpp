#ifndef INTEGRATION_TESTS_VIRTUAL_SUPPORT_VIRTUAL_ARM_FIXTURE_HPP
#define INTEGRATION_TESTS_VIRTUAL_SUPPORT_VIRTUAL_ARM_FIXTURE_HPP

#include "tools/arm_host/ArmHost.hpp"
#include "tools/virtual_arm/VirtualArm.hpp"
#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace integration
{
    class VirtualArmFixture
    {
    public:
        VirtualArmFixture();

        std::optional<std::string> Start();
        bool Command(const arm_host::ArmCommand& command);
        bool WaitFor(const std::function<bool(const arm_host::ArmSnapshot&)>& condition, std::chrono::milliseconds timeout);
        bool WaitForEvent(const std::function<bool(const arm_host::ArmEvent&)>& condition, std::chrono::milliseconds timeout);
        void ForgetTrace();
        float LargestTrackingError();

        arm::ArmProfile profile;
        std::optional<virtual_arm::VirtualArm> virtualArm;
        std::optional<arm_host::ArmHost> host;
        std::vector<arm_host::ArmEvent> events;
        std::vector<arm_host::TraceSample> trace;

    private:
        void Collect();
    };
}

#endif
