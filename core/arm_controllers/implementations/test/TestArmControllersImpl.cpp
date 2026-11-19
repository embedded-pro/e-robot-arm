#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "core/arm_controllers/test_doubles/ArmControllerMock.hpp"
#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/test_doubles/JointMotionMock.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace testing;

    class ArmControllersObserverMock
        : public arm::ArmControllersObserver
    {
    public:
        using arm::ArmControllersObserver::ArmControllersObserver;

        MOCK_METHOD(void, OnActiveControllerChanged, (std::optional<arm::ArmControllerId> active), (override));
    };

    arm::JointPositions Uniform(float value)
    {
        arm::JointPositions positions{};
        positions.fill(foc::Radians{ value });
        return positions;
    }

    arm::JointTorques Torques(float value)
    {
        arm::JointTorques torques{};
        torques.fill(foc::NewtonMeter{ value });
        return torques;
    }

    MATCHER_P(MeasuredAt, positions, "")
    {
        return arg.positions == positions;
    }

    class ArmControllersImplTest
        : public Test
    {
    public:
        ArmControllersImplTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = foc::Radians{ 0.25f };
                readings[joint].speed = foc::RadiansPerSecond{ 0.5f };
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
            EXPECT_CALL(motion, Moving()).WillRepeatedly(Invoke([this]()
                {
                    return moving;
                }));
            EXPECT_CALL(stateless, BumplessTransfer()).WillRepeatedly(Return(false));
            EXPECT_CALL(bumpless, BumplessTransfer()).WillRepeatedly(Return(true));
        }

        void SelectStateless()
        {
            controllers.Enable(statelessId);
            EXPECT_CALL(observer, OnActiveControllerChanged(std::optional<arm::ArmControllerId>{ statelessId }));
            EXPECT_TRUE(controllers.Select(statelessId));
        }

        void EngageStateless()
        {
            SelectStateless();
            EXPECT_CALL(stateless, Reset(MeasuredAt(Uniform(0.25f))));
            controllers.Engage();
        }

        StrictMock<arm::JointBusMock> bus;
        StrictMock<arm::JointMotionMock> motion;
        StrictMock<arm::ArmControllerMock> stateless;
        StrictMock<arm::ArmControllerMock> bumpless;
        std::array<arm::JointReading, arm::jointCount> readings{};
        bool moving{ false };
        arm::ArmControllersImpl controllers{ bus, motion };
        StrictMock<ArmControllersObserverMock> observer{ controllers };
        arm::ArmControllerId statelessId{ controllers.Register("stepping", stateless) };
        arm::ArmControllerId bumplessId{ controllers.Register("bumpless", bumpless) };
    };
}

TEST_F(ArmControllersImplTest, registers_controllers_disabled_and_finds_them_by_name)
{
    EXPECT_EQ(2u, controllers.Count());
    EXPECT_EQ("bumpless", controllers.Name(bumplessId));
    EXPECT_EQ(std::optional<arm::ArmControllerId>{ bumplessId }, controllers.Find("bumpless"));
    EXPECT_EQ(std::nullopt, controllers.Find("missing"));
    EXPECT_FALSE(controllers.Enabled(statelessId));
    EXPECT_EQ(std::nullopt, controllers.Active());

    EXPECT_FALSE(controllers.Select(statelessId));
    EXPECT_FALSE(controllers.Enable(7));
    EXPECT_FALSE(controllers.Disable(7));
    EXPECT_FALSE(controllers.Select(7));
}

TEST_F(ArmControllersImplTest, selects_an_enabled_controller_once_and_reports_the_change)
{
    SelectStateless();

    EXPECT_TRUE(controllers.Enabled(statelessId));
    EXPECT_EQ(std::optional<arm::ArmControllerId>{ statelessId }, controllers.Active());
    EXPECT_TRUE(controllers.Select(statelessId));
}

TEST_F(ArmControllersImplTest, while_a_move_runs_only_a_bumpless_controller_takes_over_from_the_measured_joints)
{
    controllers.Enable(bumplessId);
    EXPECT_CALL(observer, OnActiveControllerChanged(std::optional<arm::ArmControllerId>{ bumplessId }));
    EXPECT_TRUE(controllers.Select(bumplessId));
    EXPECT_CALL(bumpless, Reset(MeasuredAt(Uniform(0.25f))));
    controllers.Engage();
    moving = true;

    controllers.Enable(statelessId);
    EXPECT_FALSE(controllers.Select(statelessId));
    EXPECT_EQ(std::optional<arm::ArmControllerId>{ bumplessId }, controllers.Active());

    moving = false;
    EXPECT_CALL(stateless, Reset(MeasuredAt(Uniform(0.25f))));
    EXPECT_CALL(observer, OnActiveControllerChanged(std::optional<arm::ArmControllerId>{ statelessId }));
    EXPECT_TRUE(controllers.Select(statelessId));

    moving = true;
    EXPECT_CALL(bumpless, Reset(MeasuredAt(Uniform(0.25f))));
    EXPECT_CALL(observer, OnActiveControllerChanged(std::optional<arm::ArmControllerId>{ bumplessId }));
    EXPECT_TRUE(controllers.Select(bumplessId));
}

TEST_F(ArmControllersImplTest, streams_the_active_controllers_set_each_step_holding_the_last_reference)
{
    EXPECT_TRUE(controllers.Step(arm::AtRest(Uniform(1.0f))));

    EngageStateless();
    EXPECT_TRUE(controllers.Streaming());

    EXPECT_CALL(stateless, Step(MeasuredAt(Uniform(0.25f)), arm::AtRest(Uniform(0.25f)))).WillOnce(Return(arm::JointSetpoints{ Uniform(0.25f), Torques(0.3f) }));
    EXPECT_CALL(bus, SendTorques(Torques(0.3f))).WillOnce(Return(true));
    EXPECT_TRUE(controllers.Step(std::nullopt));

    EXPECT_CALL(stateless, Step(_, arm::AtRest(Uniform(1.0f)))).Times(2).WillRepeatedly(Return(arm::JointSetpoints{ Uniform(1.0f), Torques(0.9f) }));
    EXPECT_CALL(bus, SendTorques(Torques(0.9f))).Times(2).WillRepeatedly(Return(true));
    EXPECT_TRUE(controllers.Step(arm::AtRest(Uniform(1.0f))));
    EXPECT_TRUE(controllers.Step(std::nullopt));
    ASSERT_TRUE(controllers.LastSetpoints().has_value());
    EXPECT_EQ(Torques(0.9f), controllers.LastSetpoints()->torques);
    EXPECT_EQ(std::optional<arm::JointPositions>{ Uniform(1.0f) }, controllers.Reference());

    controllers.Disengage();
    EXPECT_TRUE(controllers.Step(arm::AtRest(Uniform(1.0f))));
    EXPECT_EQ(std::nullopt, controllers.Reference());
}

TEST_F(ArmControllersImplTest, the_active_controller_is_disabled_only_while_it_does_not_drive_the_joints)
{
    EngageStateless();

    EXPECT_FALSE(controllers.Disable(statelessId));
    EXPECT_EQ(std::optional<arm::ArmControllerId>{ statelessId }, controllers.Active());
    EXPECT_TRUE(controllers.Enabled(statelessId));
    EXPECT_TRUE(controllers.Disable(bumplessId));

    controllers.Disengage();
    EXPECT_CALL(observer, OnActiveControllerChanged(std::optional<arm::ArmControllerId>{}));
    EXPECT_TRUE(controllers.Disable(statelessId));
    EXPECT_EQ(std::nullopt, controllers.Active());
}

TEST_F(ArmControllersImplTest, streaming_without_an_active_controller_stops_at_once)
{
    controllers.Engage();

    EXPECT_FALSE(controllers.Step(arm::AtRest(Uniform(1.0f))));
    EXPECT_FALSE(controllers.Streaming());
}

TEST_F(ArmControllersImplTest, a_set_the_supervisor_refuses_stops_the_streaming)
{
    EngageStateless();

    EXPECT_CALL(stateless, Step(_, _)).WillOnce(Return(arm::JointSetpoints{ Uniform(3.0f), Torques(3.0f) }));
    EXPECT_CALL(bus, SendTorques(_)).WillOnce(Return(false));
    EXPECT_FALSE(controllers.Step(arm::AtRest(Uniform(3.0f))));

    EXPECT_FALSE(controllers.Streaming());
    EXPECT_FALSE(controllers.LastSetpoints().has_value());
}

TEST_F(ArmControllersImplTest, holding_tracks_the_measured_pose_at_rest_from_a_reset_controller)
{
    EngageStateless();
    EXPECT_CALL(stateless, Step(_, arm::AtRest(Uniform(1.0f)))).WillOnce(Return(arm::JointSetpoints{ Uniform(1.0f), Torques(0.9f) }));
    EXPECT_CALL(bus, SendTorques(Torques(0.9f))).WillOnce(Return(true));
    EXPECT_TRUE(controllers.Step(arm::AtRest(Uniform(1.0f))));

    EXPECT_CALL(stateless, Reset(MeasuredAt(Uniform(0.25f))));
    controllers.Hold();

    EXPECT_CALL(stateless, Step(_, arm::AtRest(Uniform(0.25f)))).WillOnce(Return(arm::JointSetpoints{ Uniform(0.25f), Torques(0.1f) }));
    EXPECT_CALL(bus, SendTorques(Torques(0.1f))).WillOnce(Return(true));
    EXPECT_TRUE(controllers.Step(std::nullopt));
    EXPECT_EQ(std::optional<arm::JointPositions>{ Uniform(0.25f) }, controllers.Reference());
}

namespace
{
    class ArmControllersExtrapolatingTest
        : public ArmControllersImplTest
    {
    public:
        ArmControllersExtrapolatingTest()
        {
            EXPECT_CALL(bus, Period()).WillRepeatedly(Return(9));
            for (auto& reading : readings)
                reading.period = 9;
        }

        arm::ArmControllersImpl extrapolating{ bus, motion, std::chrono::milliseconds{ 10 } };
        arm::ArmControllerId statelessHere{ extrapolating.Register("stepping", stateless) };
    };
}

TEST_F(ArmControllersExtrapolatingTest, a_reading_from_an_earlier_period_is_advanced_by_its_speed_for_each_period_it_missed)
{
    readings[2].period = 8;
    readings[4].period = 2;
    extrapolating.Enable(statelessHere);
    extrapolating.Select(statelessHere);

    EXPECT_CALL(stateless, Reset(_)).WillOnce(Invoke([](const arm::MeasuredJoints& measured)
        {
            EXPECT_NEAR(0.25f, measured.positions[0].Value(), 1e-6f);
            EXPECT_NEAR(0.25f + 0.5f * 0.01f, measured.positions[2].Value(), 1e-6f);
            EXPECT_NEAR(0.25f + 0.5f * 0.02f, measured.positions[4].Value(), 1e-6f);
            EXPECT_NEAR(0.5f, measured.speeds[2].Value(), 1e-6f);
        }));
    extrapolating.Engage();
}
