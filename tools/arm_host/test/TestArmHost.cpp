#include "tools/arm_host/ArmHost.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <future>
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
            return [](const arm_host::ArmTarget&, const arm::ArmConfig& config) -> std::unique_ptr<arm_host::ArmTargetLink>
            {
                return std::make_unique<arm_host::SimulatedTargetLink>(config.nodes, 20ms);
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

TEST_F(ArmHostTest, a_loaded_profile_rebuilds_the_arm_and_answers_with_its_id)
{
    const auto id = host->LoadProfile(arm_host::DefaultArmProfile({ 31, 32, 33, 34, 35, 36 }));

    EXPECT_TRUE(Eventually([this, id]()
        {
            for (const auto& event : host->TakeEvents())
                events.push_back(event);
            return std::ranges::any_of(events, [id](const arm_host::ArmEvent& event)
                {
                    return event.command == id && event.kind == arm_host::ArmEventKind::profileLoaded;
                });
        }));
    EXPECT_TRUE(Eventually([this]()
        {
            return host->Snapshot().profileName == "generic six-joint" && host->Snapshot().config.nodes[0] == 31;
        }));
}

TEST_F(ArmHostTest, detects_the_adapters_on_its_own_thread_and_answers_without_blocking_the_caller)
{
    std::promise<void> release;
    auto released = release.get_future().share();
    std::thread::id detectedOn{};
    std::promise<std::vector<std::string>> answer;
    auto answered = answer.get_future();

    host->DetectAdapters([&detectedOn, released]()
        {
            detectedOn = std::this_thread::get_id();
            released.wait();
            return std::vector<std::string>{ "can0", "slcan:/dev/ttyACM0" };
        },
        [&answer](std::vector<std::string> adapters)
        {
            answer.set_value(std::move(adapters));
        });

    EXPECT_EQ(std::future_status::timeout, answered.wait_for(std::chrono::milliseconds{ 50 }));
    release.set_value();
    ASSERT_EQ(std::future_status::ready, answered.wait_for(std::chrono::seconds{ 5 }));
    EXPECT_EQ((std::vector<std::string>{ "can0", "slcan:/dev/ttyACM0" }), answered.get());
    EXPECT_NE(std::this_thread::get_id(), detectedOn);
}
