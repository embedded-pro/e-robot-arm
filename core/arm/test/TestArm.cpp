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

TEST_F(ArmTest, a_move_reaches_every_joint_through_the_supervisor_in_the_setpoint_phase)
{
    AnswerCompatibleContracts();
    arm.Loop().Start();
    ReportMotionFromEveryJoint();

    arm::JointPositions target{};
    target[0] = foc::Radians{ 0.5f };
    EXPECT_TRUE(arm.Motion().MoveTo(target));

    ForwardTime(5ms);
    EXPECT_EQ(arm::jointCount, SentOf(can::SetpointCommandId(can::FocSetpointKind::position)));
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
    EXPECT_EQ(0u, SentOf(can::SetpointCommandId(can::FocSetpointKind::position)));
}
