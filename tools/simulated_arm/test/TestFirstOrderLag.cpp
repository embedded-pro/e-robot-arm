#include "tools/simulated_arm/FirstOrderLag.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <cmath>

namespace
{
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-5f };

    class FirstOrderLagTest
        : public testing::Test
    {
    public:
        simulation::JointMotionState Follow(infra::Duration elapsed, infra::Duration timeConstant) const
        {
            return simulation::FollowWithLag(foc::Radians{ 0.0f }, foc::Radians{ 1.0f }, elapsed, timeConstant);
        }
    };
}

TEST_F(FirstOrderLagTest, no_time_constant_follows_at_once_and_stands_still)
{
    const auto state = Follow(10ms, 0ms);

    EXPECT_NEAR(1.0f, state.position.Value(), tolerance);
    EXPECT_NEAR(0.0f, state.speed.Value(), tolerance);
}

TEST_F(FirstOrderLagTest, one_time_constant_covers_all_but_one_over_e)
{
    const auto state = Follow(50ms, 50ms);

    EXPECT_NEAR(1.0f - std::exp(-1.0f), state.position.Value(), tolerance);
    EXPECT_NEAR(std::exp(-1.0f) / 0.05f, state.speed.Value(), 1.0e-3f);
}

TEST_F(FirstOrderLagTest, no_elapsed_time_does_not_move)
{
    const auto state = Follow(0ms, 50ms);

    EXPECT_NEAR(0.0f, state.position.Value(), tolerance);
    EXPECT_NEAR(1.0f / 0.05f, state.speed.Value(), 1.0e-3f);
}

TEST_F(FirstOrderLagTest, settles_on_the_target)
{
    const auto state = Follow(2s, 50ms);

    EXPECT_NEAR(1.0f, state.position.Value(), tolerance);
    EXPECT_NEAR(0.0f, state.speed.Value(), 1.0e-3f);
}
