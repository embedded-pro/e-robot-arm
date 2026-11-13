#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/joint_configurator/implementations/JointConfiguratorImpl.hpp"
#include "core/joint_configurator/test_doubles/JointConfiguratorMock.hpp"
#include "core/lifecycle/test_doubles/ArmLifecycleMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    constexpr std::size_t joint{ 2 };

    class JointConfiguratorImplTest
        : public Test
        , public infra::ClockFixture
    {
    public:
        JointConfiguratorImplTest()
        {
            EXPECT_CALL(lifecycle, State()).WillRepeatedly(Invoke([this]()
                {
                    return state;
                }));
            settings.current = can::FocCurrentAlgorithm::slidingMode;
            settings.speed = can::FocSpeedAlgorithm::lqi;
            settings.position = can::FocPositionAlgorithm::twoDof;
            settings[arm::Gain::speedBandwidth] = 120.0f;
        }

        void AnswerAlgorithms(std::size_t answering, const can::FocAlgorithms& algorithms)
        {
            bus.NotifyObservers([answering, &algorithms](auto& observer)
                {
                    observer.OnAlgorithms(answering, algorithms);
                });
        }

        void AnswerParameter(std::size_t answering, can::FocParameter parameter, float value)
        {
            bus.NotifyObservers([answering, parameter, value](auto& observer)
                {
                    observer.OnParameter(answering, parameter, can::FocParameterValue{ value, false });
                });
        }

        void ExpectAndAnswerSelections()
        {
            InSequence order;
            EXPECT_CALL(bus, SelectAlgorithm(joint, can::FocControlLoop::current, static_cast<uint8_t>(settings.current))).WillOnce(Return(true));
            EXPECT_CALL(bus, SelectAlgorithm(joint, can::FocControlLoop::speed, static_cast<uint8_t>(settings.speed))).WillOnce(Return(true));
            EXPECT_CALL(bus, SelectAlgorithm(joint, can::FocControlLoop::position, static_cast<uint8_t>(settings.position))).WillOnce(Return(true));
        }

        StrictMock<arm::JointBusMock> bus;
        StrictMock<arm::ArmLifecycleMock> lifecycle;
        arm::ArmState state{ arm::ArmState::ready };
        arm::JointConfiguratorImpl configurator{ bus, lifecycle };
        StrictMock<arm::JointConfiguratorObserverMock> observer{ configurator };
        arm::ControllerSettings settings{};
    };
}

TEST_F(JointConfiguratorImplTest, refuses_unless_the_arm_is_ready)
{
    for (const auto refused : { arm::ArmState::unpowered, arm::ArmState::waitingForCalibration, arm::ArmState::enabled, arm::ArmState::moving,
             arm::ArmState::tripped })
    {
        state = refused;
        EXPECT_FALSE(configurator.Apply(joint, settings));
        EXPECT_FALSE(configurator.Read(joint));
    }
    EXPECT_FALSE(configurator.Busy());
}

TEST_F(JointConfiguratorImplTest, apply_selects_the_three_algorithms_then_sets_the_fourteen_gains_one_at_a_time)
{
    ExpectAndAnswerSelections();
    EXPECT_TRUE(configurator.Apply(joint, settings));
    EXPECT_TRUE(configurator.Busy());
    EXPECT_FALSE(configurator.Read(joint));

    AnswerAlgorithms(joint, {});
    AnswerAlgorithms(joint, {});
    EXPECT_CALL(bus, SetParameter(joint, arm::gainParameters[0], settings.gains[0])).WillOnce(Return(true));
    AnswerAlgorithms(joint, {});

    for (std::size_t gain = 1; gain != arm::gainCount; ++gain)
    {
        EXPECT_CALL(bus, SetParameter(joint, arm::gainParameters[gain], settings.gains[gain])).WillOnce(Return(true));
        AnswerParameter(joint, arm::gainParameters[gain - 1], settings.gains[gain - 1]);
    }

    EXPECT_CALL(observer, OnApplied(joint));
    AnswerParameter(joint, arm::gainParameters.back(), settings.gains.back());
    EXPECT_FALSE(configurator.Busy());
}

TEST_F(JointConfiguratorImplTest, read_collects_the_mode_the_algorithms_and_every_gain)
{
    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(true));
    EXPECT_TRUE(configurator.Read(joint));

    EXPECT_CALL(bus, QueryParameter(joint, _)).Times(arm::gainCount).WillRepeatedly(Return(true));
    AnswerAlgorithms(joint, { can::FocMotorMode::position, can::FocCurrentAlgorithm::deadbeat, can::FocSpeedAlgorithm::adrc, can::FocPositionAlgorithm::lqi });

    EXPECT_CALL(observer, OnRead(joint, _)).WillOnce(Invoke([](std::size_t, const arm::ControllerReading& reading)
        {
            EXPECT_EQ(can::FocMotorMode::position, reading.mode);
            EXPECT_EQ(can::FocCurrentAlgorithm::deadbeat, reading.settings.current);
            EXPECT_EQ(can::FocSpeedAlgorithm::adrc, reading.settings.speed);
            EXPECT_EQ(can::FocPositionAlgorithm::lqi, reading.settings.position);
            for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
                EXPECT_NEAR(static_cast<float>(gain) + 0.5f, reading.settings.gains[gain], 1.0e-6f);
        }));
    for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
        AnswerParameter(joint, arm::gainParameters[gain], static_cast<float>(gain) + 0.5f);
}

TEST_F(JointConfiguratorImplTest, answers_for_another_joint_or_another_parameter_do_not_advance)
{
    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(true));
    EXPECT_TRUE(configurator.Read(joint));

    AnswerAlgorithms(joint + 1, {});
    AnswerParameter(joint, can::FocParameter::currentBandwidth, 1.0f);

    EXPECT_CALL(bus, QueryParameter(joint, arm::gainParameters[0])).WillOnce(Return(true));
    AnswerAlgorithms(joint, {});

    AnswerParameter(joint, arm::gainParameters[1], 1.0f);
    EXPECT_CALL(bus, QueryParameter(joint, arm::gainParameters[1])).WillOnce(Return(true));
    AnswerParameter(joint, arm::gainParameters[0], 1.0f);
}

TEST_F(JointConfiguratorImplTest, a_request_the_queue_refuses_is_sent_again_a_millisecond_later)
{
    InSequence order;
    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(false));
    EXPECT_TRUE(configurator.Read(joint));

    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(false));
    ForwardTime(1ms);
    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(true));
    ForwardTime(1ms);
    ForwardTime(10ms);
}

TEST_F(JointConfiguratorImplTest, a_request_left_unanswered_fails_the_operation_after_100_ms)
{
    EXPECT_CALL(bus, SelectAlgorithm(joint, can::FocControlLoop::current, _)).WillOnce(Return(true));
    EXPECT_CALL(bus, SelectAlgorithm(joint, can::FocControlLoop::speed, _)).WillOnce(Return(true));
    EXPECT_TRUE(configurator.Apply(joint, settings));
    AnswerAlgorithms(joint, {});

    ForwardTime(99ms);
    EXPECT_CALL(observer, OnConfigurationFailed(joint, arm::ConfigurationFailure::timedOut));
    ForwardTime(1ms);

    EXPECT_FALSE(configurator.Busy());
    AnswerAlgorithms(joint, {});
    EXPECT_CALL(bus, QueryAlgorithms(joint)).WillOnce(Return(true));
    EXPECT_TRUE(configurator.Read(joint));
}
