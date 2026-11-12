#ifndef TOOLS_ARM_HOST_ADAPTER_TARGET_LINK_HPP
#define TOOLS_ARM_HOST_ADAPTER_TARGET_LINK_HPP

#include "can-lite/drivers/hub/VirtualCanHub.hpp"
#include "can-lite/drivers/implementation/PlatformCanAdapter.hpp"
#include "tools/arm_host/ArmTargetLink.hpp"
#include <cstdint>
#include <memory>

namespace arm_host
{
    class AdapterTargetLink
        : public ArmTargetLink
    {
    public:
        static std::unique_ptr<ArmTargetLink> Open(const ArmTarget& target, uint32_t bitrate);

        AdapterTargetLink(const ArmTarget& target, uint32_t bitrate);
        ~AdapterTargetLink() override;

        hal::Can& Can() override;
        void Poll() override;
        simulation::SimulatedArm* Simulation() override;

    private:
        services::VirtualCanHub hub;
        services::PlatformCanAdapter platformAdapter;
        bool connected{ false };
    };

    TargetOpener HostTargetOpener(const arm::JointNodeIds& nodes, infra::Duration simulatedTimeConstant, uint32_t bitrate);
}

#endif
