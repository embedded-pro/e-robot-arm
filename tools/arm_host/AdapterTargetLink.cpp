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
        : bitrate(bitrate)
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

    bool AdapterTargetLink::Connected()
    {
        return platformAdapter.Adapter().IsConnected();
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

    std::vector<std::string> DetectedAdapters()
    {
        services::PlatformCanAdapter platform;
        std::vector<std::string> adapters;
        platform.Adapter().EnumerateInterfaces([&adapters](infra::BoundedConstString name)
            {
                std::string adapter{ name.begin(), name.end() };
                if (!adapter.starts_with("virtual:"))
                    adapters.push_back(std::move(adapter));
            });
        return adapters;
    }

    TargetOpener HostTargetOpener(infra::Duration simulatedTimeConstant, uint32_t bitrate)
    {
        return [simulatedTimeConstant, bitrate](const ArmTarget& target, const arm::ArmConfig& config) -> std::unique_ptr<ArmTargetLink>
        {
            if (target.kind == TargetKind::simulated)
                return std::make_unique<SimulatedTargetLink>(config, simulatedTimeConstant);
            if (target.kind == TargetKind::virtualArm)
                return nullptr;
            return AdapterTargetLink::Open(target, bitrate);
        };
    }

    uint32_t AdapterTargetLink::Bitrate() const
    {
        return bitrate;
    }
}
