#include "tools/arm_host/ArmHost.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <thread>

namespace
{
    using namespace std::chrono_literals;

    class ArmHostTest
        : public testing::Test
    {
    public:
        template<class Predicate>
        bool Eventually(Predicate predicate)
        {
            const auto deadline = std::chrono::steady_clock::now() + 2s;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (predicate())
                    return true;
                std::this_thread::sleep_for(5ms);
            }
            return false;
        }

        static arm_host::TargetOpener Opener()
        {
            return [](const arm_host::ArmTarget&, const arm::JointNodeIds& nodes) -> std::unique_ptr<arm_host::ArmTargetLink>
            {
                return std::make_unique<arm_host::SimulatedTargetLink>(nodes, 20ms);
            };
        }

        std::optional<arm_host::ArmHost> host{ std::in_place, arm_host::test::Profile(), Opener(), arm_host::ArmTarget{} };
        std::vector<arm_host::ArmEvent> events;
    };
}

TEST_F(ArmHostTest, publishes_snapshots_from_its_own_thread)
{
    EXPECT_TRUE(Eventually([this]()
        {
            return host->Snapshot().sequence > 5;
        }));
    EXPECT_TRUE(host->Snapshot().targetReady);
    EXPECT_FALSE(host->TakeTrace().empty());
}

TEST_F(ArmHostTest, a_posted_command_answers_with_its_id)
{
    const auto id = host->Post(arm_host::CheckContracts{});

    EXPECT_TRUE(Eventually([this, id]()
        {
            for (const auto& event : host->TakeEvents())
                events.push_back(event);
            return std::ranges::any_of(events, [id](const arm_host::ArmEvent& event)
                {
                    return event.command == id && event.kind == arm_host::ArmEventKind::accepted;
                });
        }));
}

TEST_F(ArmHostTest, shuts_down_with_commands_still_queued)
{
    for (int command = 0; command != 50; ++command)
        host->Post(arm_host::CheckContracts{});

    host.reset();

    EXPECT_FALSE(host.has_value());
}
