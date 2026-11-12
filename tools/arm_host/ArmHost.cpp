#include "tools/arm_host/ArmHost.hpp"

namespace arm_host
{
    void ArmRequestQueue::Push(ArmRequest request)
    {
        const auto* command = std::get_if<ArmCommand>(&request.what);
        const bool first = command != nullptr && std::holds_alternative<EmergencyStop>(*command);

        std::scoped_lock lock{ mutex };
        if (first)
            requests.push_front(std::move(request));
        else
            requests.push_back(std::move(request));
    }

    std::optional<ArmRequest> ArmRequestQueue::Pop()
    {
        std::scoped_lock lock{ mutex };
        if (requests.empty())
            return std::nullopt;

        auto request = std::move(requests.front());
        requests.pop_front();
        return request;
    }

    ArmHost::ArmHost(const arm::ArmConfig& config, const TargetOpener& opener, const ArmTarget& initial)
    {
        runtime.emplace(config, opener, mailbox, initial);
        thread = std::thread{ [this]()
            {
                dispatcher.ExecuteUntil([this]()
                    {
                        return stopping.load();
                    });
                runtime.reset();
            } };
    }

    ArmHost::~ArmHost()
    {
        dispatcher.Schedule([this]()
            {
                stopping = true;
            });
        thread.join();
    }

    uint32_t ArmHost::Post(const ArmCommand& command)
    {
        return Enqueue(command);
    }

    uint32_t ArmHost::SwitchTarget(const ArmTarget& target)
    {
        return Enqueue(target);
    }

    ArmSnapshot ArmHost::Snapshot() const
    {
        return mailbox.Snapshot();
    }

    std::vector<ArmEvent> ArmHost::TakeEvents()
    {
        return mailbox.TakeEvents();
    }

    std::vector<TraceSample> ArmHost::TakeTrace()
    {
        return mailbox.TakeTrace();
    }

    uint32_t ArmHost::Enqueue(std::variant<ArmCommand, ArmTarget> what)
    {
        const auto id = nextId++;
        requests.Push({ id, std::move(what) });

        dispatcher.Schedule([this]()
            {
                RunOne();
            });
        return id;
    }

    void ArmHost::RunOne()
    {
        const auto request = requests.Pop();
        if (!request)
            return;

        if (const auto* command = std::get_if<ArmCommand>(&request->what))
            runtime->Execute(request->id, *command);
        else
            runtime->SwitchTarget(request->id, std::get<ArmTarget>(request->what));
    }
}
