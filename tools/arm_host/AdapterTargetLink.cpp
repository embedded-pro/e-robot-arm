#include "tools/arm_host/AdapterTargetLink.hpp"

namespace arm_host
{
    std::unique_ptr<ArmTargetLink> AdapterTargetLink::Open(const ArmTarget& target, uint32_t bitrate)
    {
        auto link = std::make_unique<AdapterTargetLink>(target, bitrate);
        if (!link->connected)
            return nullptr;
        return link;
    }

    AdapterTargetLink::AdapterTargetLink(const ArmTarget& target, uint32_t bitrate)
    {
        if (target.kind == TargetKind::virtualBus)
            hub.Listen();

        connected = platformAdapter.Adapter().Connect(target.bus, bitrate);
    }

    AdapterTargetLink::~AdapterTargetLink()
    {
        if (connected)
            platformAdapter.Adapter().Disconnect();
    }

    hal::Can& AdapterTargetLink::Can()
    {
        return platformAdapter.Adapter();
    }

    void AdapterTargetLink::Poll()
    {
        hub.Poll();
        platformAdapter.Adapter().ProcessReadEvent();
    }

    simulation::SimulatedArm* AdapterTargetLink::Simulation()
    {
        return nullptr;
    }

    TargetOpener HostTargetOpener(infra::Duration simulatedTimeConstant, uint32_t bitrate)
    {
        return [simulatedTimeConstant, bitrate](const ArmTarget& target, const arm::JointNodeIds& nodes) -> std::unique_ptr<ArmTargetLink>
        {
            if (target.kind == TargetKind::simulated)
                return std::make_unique<SimulatedTargetLink>(nodes, simulatedTimeConstant);
            return AdapterTargetLink::Open(target, bitrate);
        };
    }
}
