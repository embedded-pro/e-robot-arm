#include "can-lite/core/CanPayload.hpp"
#include "can-lite/core/CanProtocolDefinitions.hpp"
#include "can-lite/core/test/CanMock.hpp"
#include "core/arm/Arm.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "e-foc-protocol/contract/FocMotorWireContract.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    constexpr arm::JointNodeIds nodes{ 21, 22, 23, 24, 25, 26 };

    arm::ArmConfig Config()
    {
        arm::ArmConfig config{ nodes, 3, {}, {}, 10ms };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
        for (auto& limit : config.motionLimits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
        return config;
    }

    struct SentFrame
    {
        uint16_t node;
        uint8_t messageType;
        services::CanPriority priority;
    };

    class ArmTest
        : public Test
        , public infra::ClockFixture
    {
    public:
        void Receive(uint16_t node, uint8_t messageType, const services::CanPayloadWriter& payload)
        {
            receive(hal::Can::Id::Create29BitId(services::MakeCanId(services::CanPriority::response, can::focMotorCategoryId, messageType, node)), payload.Message());
        }

        void AnswerCompatibleContracts()
        {
            for (const auto node : nodes)
                Receive(node, can::focContractVersionResponseId, services::CanPayloadWriter{}.WriteUInt8(can::focContractVersionMajor).WriteUInt8(can::focContractVersionMinor));
        }

        void ReportMotionFromEveryJoint()
        {
            for (const auto node : nodes)
                Receive(node, can::focTelemetryMotionId, services::CanPayloadWriter{}.WriteFloat32(0.0f).WriteFloat32(0.0f));
        }

        void ReportStatus(uint16_t node, can::FocMotorState state, can::FocFaultCode fault)
        {
            Receive(node, can::focTelemetryStatusResponseId, services::CanPayloadWriter{}.WriteUInt8(static_cast<uint8_t>(state)).WriteUInt8(static_cast<uint8_t>(fault)));
        }

        void EnableArm()
        {
            arm.Loop().Start();
            EXPECT_TRUE(arm.Lifecycle().BringUp());
            AnswerCompatibleContracts();
            for (int period = 0; period != 4; ++period)
            {
                for (const auto node : nodes)
                    ReportStatus(node, can::FocMotorState::running, can::FocFaultCode::none);
                ReportMotionFromEveryJoint();
                AnswerParameterRequests();
                ForwardTime(10ms);
            }
            ReportMotionFromEveryJoint();
            ASSERT_EQ(arm::ArmState::enabled, arm.Lifecycle().State());
        }

        void AnswerParameterRequests()
        {
            while (answered != sent.size())
            {
                const auto frame = sent[answered++];
                if (frame.messageType == can::focSetParameterId || frame.messageType == can::focQueryParameterId)
                    Receive(frame.node, can::focParameterResponseId,
                        services::CanPayloadWriter{}.WriteUInt8(static_cast<uint8_t>(can::FocParameter::commandTimeout)).WriteUInt8(0).WriteFloat32(0.1f));
            }
        }

        std::size_t SentOf(uint8_t messageType) const
        {
            return static_cast<std::size_t>(std::ranges::count_if(sent, [messageType](const SentFrame& frame)
                {
                    return frame.messageType == messageType;
                }));
        }

        StrictMock<hal::CanMock> canMock;
        infra::Function<void(hal::Can::Id, const hal::Can::Message&)> receive;
        std::vector<SentFrame> sent;
        std::size_t answered{ 0 };
        infra::Execute setupBus{ [this]()
            {
                EXPECT_CALL(canMock, ReceiveData(_)).WillRepeatedly(SaveArg<0>(&receive));
                EXPECT_CALL(canMock, SendData(_, _, _))
                    .WillRepeatedly(Invoke([this](hal::Can::Id id, const hal::Can::Message&, const infra::Function<void(bool)>& onDone)
                        {
                            const auto canId = id.Get29BitId();
                            sent.push_back({ services::ExtractCanNodeId(canId), services::ExtractCanMessageType(canId), services::ExtractCanPriority(canId) });
                            onDone(true);
                        }));
            } };
        can::FocMotorCanClient client{ canMock };
        arm::Arm arm{ client.CategoryClient(), Config() };
    };
}

TEST_F(ArmTest, the_running_loop_requests_telemetry_from_every_joint_each_period)
{
    arm.Loop().Start();
    EXPECT_EQ(arm::jointCount, SentOf(can::focRequestTelemetryId));

    ForwardTime(10ms);
    EXPECT_EQ(2 * arm::jointCount, SentOf(can::focRequestTelemetryId));
}

TEST_F(ArmTest, an_enabled_arm_streams_the_active_controllers_setpoints_to_every_joint_each_period)
{
    EXPECT_EQ(0u, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
    EnableArm();
    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));

    ForwardTime(10ms);
    EXPECT_EQ(streamed + arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
    EXPECT_EQ(std::optional<arm::ArmControllerId>{ 0 }, arm.Controllers().Active());
}

TEST_F(ArmTest, a_move_reaches_every_joint_through_the_supervisor_in_the_setpoint_phase)
{
    EnableArm();
    ForwardTime(5ms);
    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));

    arm::JointPositions target{};
    target[0] = foc::Radians{ 0.5f };
    EXPECT_TRUE(arm.Lifecycle().MoveTo(target));

    ForwardTime(10ms);
    EXPECT_EQ(streamed + arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
    ASSERT_TRUE(arm.Controllers().LastSetpoints().has_value());
    EXPECT_GT(arm.Controllers().LastSetpoints()->positions[0].Value(), 0.0f);
}

TEST_F(ArmTest, the_controller_that_drives_the_joints_cannot_be_disabled)
{
    EnableArm();
    arm::JointPositions target{};
    target[0] = foc::Radians{ 0.5f };
    EXPECT_TRUE(arm.Lifecycle().MoveTo(target));
    ForwardTime(10ms);
    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));

    EXPECT_FALSE(arm.Controllers().Disable(0));
    ForwardTime(10ms);

    EXPECT_TRUE(arm.Motion().Moving());
    EXPECT_EQ(streamed + arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
}

TEST_F(ArmTest, a_faulted_joint_stops_every_joint)
{
    ReportStatus(nodes[3], can::FocMotorState::fault, can::FocFaultCode::overCurrent);

    EXPECT_EQ(arm::jointCount, SentOf(can::focEmergencyStopId));
    EXPECT_TRUE(arm.Safety().Tripped());
}

TEST_F(ArmTest, a_target_outside_the_joint_limits_is_refused_before_anything_moves)
{
    AnswerCompatibleContracts();
    arm.Loop().Start();

    arm::JointPositions target{};
    target[2] = foc::Radians{ 4.0f };
    EXPECT_FALSE(arm.Motion().MoveTo(target));

    ForwardTime(5ms);
    EXPECT_EQ(0u, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
}

TEST_F(ArmTest, enables_the_joints_in_torque_mode_and_streams_the_gravity_compensated_torques)
{
    const auto gravityPd = arm.Controllers().Find("gravity-pd");
    ASSERT_TRUE(gravityPd.has_value());
    EXPECT_EQ(gravityPd, arm.Controllers().Active());
    EXPECT_EQ(6u, arm.Controllers().Count());
    ASSERT_TRUE(arm.Controllers().Find("operational-space").has_value());
    EXPECT_TRUE(arm.Controllers().Enabled(*arm.Controllers().Find("operational-space")));
    ASSERT_TRUE(arm.Controllers().Find("gravity-pid").has_value());
    const auto impedance = arm.Controllers().Find("joint-impedance");
    ASSERT_TRUE(impedance.has_value());
    EXPECT_TRUE(arm.Controllers().Enabled(*impedance));
    const auto cartesian = arm.Controllers().Find("cartesian-impedance");
    ASSERT_TRUE(cartesian.has_value());
    EXPECT_TRUE(arm.Controllers().Enabled(*cartesian));
    EXPECT_FLOAT_EQ(1.0f, arm.ImpedanceShare());
    EXPECT_TRUE(arm.SetImpedanceShare(0.3f));
    EXPECT_FLOAT_EQ(0.3f, arm.ImpedanceShare());
    EXPECT_FALSE(arm.SetImpedanceShare(2.0f));
    EXPECT_FLOAT_EQ(0.3f, arm.ImpedanceShare());
    const auto computedTorque = arm.Controllers().Find("computed-torque");
    ASSERT_TRUE(computedTorque.has_value());
    EXPECT_TRUE(arm.Controllers().Enabled(*computedTorque));

    EnableArm();
    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));
    ForwardTime(10ms);

    EXPECT_EQ(streamed + arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
    EXPECT_EQ(0u, SentOf(can::SetpointCommandId(can::FocSetpointKind::position)));
}

TEST_F(ArmTest, setpoints_stream_while_the_joints_enable_so_a_joint_running_first_is_held)
{
    arm.Loop().Start();
    EXPECT_TRUE(arm.Lifecycle().BringUp());
    AnswerCompatibleContracts();
    for (int period = 0; period != 4; ++period)
    {
        for (const auto node : nodes)
            ReportStatus(node, can::FocMotorState::ready, can::FocFaultCode::none);
        ReportMotionFromEveryJoint();
        AnswerParameterRequests();
        ForwardTime(10ms);
    }
    ASSERT_EQ(arm::ArmState::ready, arm.Lifecycle().State());

    ASSERT_TRUE(arm.Lifecycle().Enable());
    for (const auto node : nodes)
        Receive(node, can::focSelectControlModeResponseId, services::CanPayloadWriter{}.WriteUInt8(static_cast<uint8_t>(can::FocMotorMode::torque)));
    ForwardTime(10ms);
    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));
    for (int period = 0; period != 3; ++period)
    {
        for (const auto node : nodes)
            ReportStatus(node, node == nodes[0] ? can::FocMotorState::running : can::FocMotorState::ready, can::FocFaultCode::none);
        ReportMotionFromEveryJoint();
        ForwardTime(10ms);
    }

    EXPECT_EQ(arm::ArmState::enabling, arm.Lifecycle().State());
    EXPECT_EQ(streamed + 3 * arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
}

TEST_F(ArmTest, gravity_pd_gains_follow_the_arms_inertia_with_each_rotor_through_its_gear)
{
    auto config = Config();
    config.transmissions[1].ratio = 50.0f;
    config.motors[2] = arm::JointMotor{ 1, arm::motorPresets[1].settings };
    config.gravityPd = { 20.0f, 0.5f };
    const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };

    const auto gains = arm::GravityPdGains(model, config);

    const auto mass = model.Dynamics().ComputeMassMatrix(model.ModelVector(arm::JointPositions{}));
    const auto teknic = arm::motorPresets[0].settings[arm::MotorParameter::inertia];
    const auto anaheim = arm::motorPresets[1].settings[arm::MotorParameter::inertia];
    EXPECT_NEAR((mass.at(0, 0) + teknic) * 400.0f, gains.kp.at(0, 0), 1.0e-3f);
    EXPECT_NEAR((mass.at(1, 1) + teknic * 2500.0f) * 400.0f, gains.kp.at(1, 1), 1.0e-3f);
    EXPECT_NEAR(mass.at(1, 2) * 400.0f, gains.kp.at(1, 2), 1.0e-3f);
    EXPECT_NEAR((mass.at(2, 2) + anaheim) * 20.0f, gains.kd.at(2, 2), 1.0e-4f);
}

TEST_F(ArmTest, the_collision_observer_waits_until_every_joint_has_reported_again)
{
    EnableArm();
    arm::JointPositions target{};
    target[1] = foc::Radians{ 0.5f };
    ASSERT_TRUE(arm.Lifecycle().MoveTo(target));
    for (int period = 0; period != 3; ++period)
    {
        ReportMotionFromEveryJoint();
        ForwardTime(10ms);
    }
    ASSERT_TRUE(arm.EstimatedExternalTorques().has_value());
    const auto held = *arm.EstimatedExternalTorques();

    for (const auto node : nodes)
        if (node != nodes[1])
            Receive(node, can::focTelemetryMotionId, services::CanPayloadWriter{}.WriteFloat32(0.0f).WriteFloat32(0.0f));
    ForwardTime(10ms);

    ASSERT_TRUE(arm.EstimatedExternalTorques().has_value());
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_EQ(held[joint].Value(), (*arm.EstimatedExternalTorques())[joint].Value()) << joint;

    ReportMotionFromEveryJoint();
    ForwardTime(10ms);

    EXPECT_NE(held[1].Value(), (*arm.EstimatedExternalTorques())[1].Value());
}

TEST_F(ArmTest, a_supervision_rule_holds_the_measured_pose_and_keeps_the_joints_powered)
{
    EnableArm();
    for (const auto node : nodes)
        Receive(node, can::focTelemetryMotionId, services::CanPayloadWriter{}.WriteFloat32(0.0f).WriteFloat32(node == nodes[0] ? 3.5f : 0.0f));
    ForwardTime(10ms);

    EXPECT_EQ(arm::ArmState::holding, arm.Lifecycle().State());
    EXPECT_TRUE(arm.Safety().Holding());
    EXPECT_EQ(0u, SentOf(can::focEmergencyStopId));

    const auto streamed = SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter));
    ForwardTime(10ms);
    EXPECT_EQ(streamed + arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::torqueNewtonMeter)));
    ASSERT_TRUE(arm.Controllers().Reference().has_value());
    EXPECT_NEAR(3.5f, (*arm.Controllers().Reference())[0].Value(), 1e-5f);
}

TEST_F(ArmTest, supervision_limits_come_from_the_profile_or_default_from_motion_and_motor)
{
    auto config = Config();
    config.speedTrips[1] = foc::RadiansPerSecond{ 2.5f };
    config.torqueLimits[2] = foc::NewtonMeter{ 12.0f };
    config.motors[3] = arm::JointMotor{ 0, arm::motorPresets[0].settings };
    config.transmissions[3].ratio = 50.0f;

    const auto supervision = arm::Supervision(config);

    EXPECT_NEAR(1.0f, supervision.reference[0].speed.Value(), 1e-6f);
    EXPECT_NEAR(2.0f, supervision.reference[0].acceleration.Value(), 1e-6f);
    EXPECT_NEAR(4.0f, supervision.speed[0].Value(), 1e-6f);
    EXPECT_NEAR(2.5f, supervision.speed[1].Value(), 1e-6f);
    EXPECT_NEAR(12.0f, supervision.torque[2].Value(), 1e-6f);
    EXPECT_NEAR(1.5f * 4.0f * 0.0064f * 20.0f * 50.0f, supervision.torque[3].Value(), 1e-3f);
    EXPECT_TRUE(std::isinf(supervision.torque[0].Value()));
}

TEST_F(ArmTest, bring_up_writes_and_reads_back_every_drives_command_timeout_before_the_arm_is_ready)
{
    EnableArm();

    EXPECT_EQ(arm::jointCount, SentOf(can::focSetParameterId));
    EXPECT_EQ(arm::jointCount, SentOf(can::focQueryParameterId));
}
