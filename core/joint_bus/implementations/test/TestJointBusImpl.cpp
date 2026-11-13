#include "can-lite/core/CanPayload.hpp"
#include "can-lite/core/CanProtocolDefinitions.hpp"
#include "can-lite/core/test/CanMock.hpp"
#include "core/joint_bus/implementations/JointBusImpl.hpp"
#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "e-foc-protocol/contract/FocMotorWireContract.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include <algorithm>
#include <vector>

namespace
{
    using namespace testing;

    constexpr arm::JointNodeIds nodes{ 11, 12, 13, 14, 15, 16 };
    constexpr std::size_t gearedJoint{ 2 };

    arm::JointTransmissions Transmissions()
    {
        arm::JointTransmissions transmissions{};
        transmissions[gearedJoint] = { 10.0f, arm::JointDirection::negative, foc::Radians{ 0.5f } };
        return transmissions;
    }

    constexpr uint16_t foreignNode{ 99 };
    constexpr float tolerance{ 1.0e-6f };
    constexpr uint8_t missedRequestsBeforeSilent{ 3 };
    constexpr infra::Duration commandTimeout{ std::chrono::milliseconds{ 30 } };

    struct SentFrame
    {
        uint16_t node;
        uint8_t messageType;
        services::CanPriority priority;
        hal::Can::Message payload;
    };

    class JointBusImplTest
        : public Test
        , public infra::ClockFixture
    {
    public:
        void Receive(uint16_t node, uint8_t messageType, const services::CanPayloadWriter& payload)
        {
            receive(hal::Can::Id::Create29BitId(services::MakeCanId(services::CanPriority::response, can::focMotorCategoryId, messageType, node)), payload.Message());
        }

        void AnswerContract(uint16_t node, uint8_t major, uint8_t minor)
        {
            Receive(node, can::focContractVersionResponseId, services::CanPayloadWriter{}.WriteUInt8(major).WriteUInt8(minor));
        }

        void AnswerCompatible(uint16_t node)
        {
            AnswerContract(node, can::focContractVersionMajor, can::focContractVersionMinor);
        }

        void AnswerIncompatible(uint16_t node)
        {
            AnswerContract(node, can::focContractVersionMajor + 1, 0);
        }

        void ReportStatus(uint16_t node, can::FocMotorState state, can::FocFaultCode fault)
        {
            Receive(node, can::focTelemetryStatusResponseId, services::CanPayloadWriter{}.WriteUInt8(static_cast<uint8_t>(state)).WriteUInt8(static_cast<uint8_t>(fault)));
        }

        void ReportMotion(uint16_t node, float speed, float position)
        {
            Receive(node, can::focTelemetryMotionId, services::CanPayloadWriter{}.WriteFloat32(speed).WriteFloat32(position));
        }

        void CompleteTransmissions()
        {
            while (!inFlight.empty())
            {
                auto onDone = inFlight.front();
                inFlight.erase(inFlight.begin());
                onDone(true);
            }
        }

        std::size_t FramesTo(uint16_t node, uint8_t messageType) const
        {
            return static_cast<std::size_t>(std::ranges::count_if(sent, [node, messageType](const SentFrame& frame)
                {
                    return frame.node == node && frame.messageType == messageType;
                }));
        }

        void ExpectOneFramePerJoint(uint8_t messageType) const
        {
            ASSERT_EQ(arm::jointCount, sent.size());
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                EXPECT_EQ(nodes[joint], sent[joint].node);
                EXPECT_EQ(messageType, sent[joint].messageType);
            }
        }

        StrictMock<hal::CanMock> canMock;
        infra::Function<void(hal::Can::Id, const hal::Can::Message&)> receive;
        std::vector<SentFrame> sent;
        bool holdTransmissions{ false };
        std::vector<infra::Function<void(bool)>> inFlight;
        infra::Execute setupBus{ [this]()
            {
                EXPECT_CALL(canMock, ReceiveData(_)).WillRepeatedly(SaveArg<0>(&receive));
                EXPECT_CALL(canMock, SendData(_, _, _))
                    .WillRepeatedly(Invoke([this](hal::Can::Id id, const hal::Can::Message& data, const infra::Function<void(bool)>& onDone)
                        {
                            const auto canId = id.Get29BitId();
                            sent.push_back({ services::ExtractCanNodeId(canId), services::ExtractCanMessageType(canId), services::ExtractCanPriority(canId), data });
                            if (holdTransmissions)
                                inFlight.push_back(onDone);
                            else
                                onDone(true);
                        }));
            } };
        can::FocMotorCanClient client{ canMock };
        arm::JointBusImpl bus{ client.CategoryClient(), nodes, Transmissions(), missedRequestsBeforeSilent, commandTimeout };
        StrictMock<arm::JointBusObserverMock> observer{ bus };
    };
}

TEST_F(JointBusImplTest, selects_the_control_mode_of_every_joint_in_joint_order)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::selectMode));
    EXPECT_TRUE(bus.SelectMode(can::FocMotorMode::position));

    ExpectOneFramePerJoint(can::focSelectControlModeId);
    for (const auto& frame : sent)
    {
        services::CanPayloadReader reader{ frame.payload };
        reader.Skip(1);
        EXPECT_EQ(static_cast<uint8_t>(can::FocMotorMode::position), reader.ReadUInt8());
    }
}

TEST_F(JointBusImplTest, start_stop_and_clear_fault_reach_every_joint)
{
    struct Case
    {
        bool (arm::JointBus::*send)();
        uint8_t messageType;
        arm::JointCommand command;
    };

    const std::array<Case, 3> cases{ { { &arm::JointBus::Start, can::focStartId, arm::JointCommand::start },
        { &arm::JointBus::Stop, can::focStopId, arm::JointCommand::stop },
        { &arm::JointBus::ClearFault, can::focClearFaultId, arm::JointCommand::clearFault } } };

    for (const auto& [send, messageType, command] : cases)
    {
        sent.clear();
        EXPECT_CALL(observer, OnCommandDelivered(command));
        EXPECT_TRUE((bus.*send)());
        ExpectOneFramePerJoint(messageType);
    }
}

TEST_F(JointBusImplTest, emergency_stop_reaches_every_joint_at_emergency_priority)
{
    EXPECT_TRUE(bus.EmergencyStop());

    ExpectOneFramePerJoint(can::focEmergencyStopId);
    for (const auto& frame : sent)
        EXPECT_EQ(services::CanPriority::emergency, frame.priority);
}

TEST_F(JointBusImplTest, sends_one_position_setpoint_per_joint_in_motor_space)
{
    const arm::JointPositions positions{ foc::Radians{ 0.1f }, foc::Radians{ -0.2f }, foc::Radians{ 0.9f },
        foc::Radians{ -0.4f }, foc::Radians{ 0.5f }, foc::Radians{ -0.6f } };
    const std::array<float, arm::jointCount> motorPositions{ 0.1f, -0.2f, -4.0f, -0.4f, 0.5f, -0.6f };

    EXPECT_TRUE(bus.SendPositions(positions));

    ExpectOneFramePerJoint(can::SetpointCommandId(can::FocSetpointKind::position));
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        services::CanPayloadReader reader{ sent[joint].payload };
        reader.Skip(1);
        EXPECT_NEAR(motorPositions[joint], reader.ReadFloat32(), tolerance) << "joint " << joint;
    }
}

TEST_F(JointBusImplTest, a_refused_joint_fails_the_emergency_stop_but_the_others_are_still_stopped)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::queryContracts));
    EXPECT_TRUE(bus.QueryContracts());
    EXPECT_CALL(observer, OnContractsResolved(can::ContractCompatibility::incompatible));
    AnswerIncompatible(nodes[2]);
    sent.clear();

    EXPECT_FALSE(bus.EmergencyStop());

    ASSERT_EQ(arm::jointCount - 1, sent.size());
    EXPECT_EQ(nodes[3], sent[2].node);
}

TEST_F(JointBusImplTest, contract_check_resolves_compatible_after_all_six_joints_answer)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::queryContracts));
    EXPECT_TRUE(bus.QueryContracts());
    ExpectOneFramePerJoint(can::focQueryContractVersionId);

    for (std::size_t joint = 0; joint != arm::jointCount - 1; ++joint)
        AnswerCompatible(nodes[joint]);
    EXPECT_EQ(can::ContractCompatibility::unknown, bus.Contracts());

    EXPECT_CALL(observer, OnContractsResolved(can::ContractCompatibility::compatible));
    AnswerCompatible(nodes.back());
    EXPECT_EQ(can::ContractCompatibility::compatible, bus.Contracts());
}

TEST_F(JointBusImplTest, first_incompatible_answer_resolves_the_contract_check_once)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::queryContracts));
    EXPECT_TRUE(bus.QueryContracts());
    AnswerCompatible(nodes[0]);

    EXPECT_CALL(observer, OnContractsResolved(can::ContractCompatibility::incompatible));
    AnswerIncompatible(nodes[1]);

    for (std::size_t joint = 2; joint != arm::jointCount; ++joint)
        AnswerCompatible(nodes[joint]);
    EXPECT_EQ(can::ContractCompatibility::incompatible, bus.Contracts());
}

TEST_F(JointBusImplTest, telemetry_updates_the_reading_of_its_joint)
{
    ReportMotion(nodes[4], 1.5f, -0.75f);
    ReportStatus(nodes[4], can::FocMotorState::running, can::FocFaultCode::none);

    const auto& reading = bus.Reading(4);
    EXPECT_NEAR(1.5f, reading.speed.Value(), tolerance);
    EXPECT_NEAR(-0.75f, reading.position.Value(), tolerance);
    EXPECT_EQ(can::FocMotorState::running, reading.state);
    EXPECT_NEAR(0.0f, bus.Reading(3).position.Value(), tolerance);
}

TEST_F(JointBusImplTest, motion_telemetry_of_a_geared_joint_is_converted_to_joint_space)
{
    ReportMotion(nodes[gearedJoint], -3.0f, -4.0f);

    EXPECT_NEAR(0.3f, bus.Reading(gearedJoint).speed.Value(), tolerance);
    EXPECT_NEAR(0.9f, bus.Reading(gearedJoint).position.Value(), tolerance);
}

TEST_F(JointBusImplTest, frames_from_nodes_outside_the_map_are_ignored)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::queryContracts));
    EXPECT_TRUE(bus.QueryContracts());
    for (const auto node : nodes)
        AnswerCompatible(node == nodes.back() ? foreignNode : node);

    ReportMotion(foreignNode, 1.0f, 1.0f);
    ReportStatus(foreignNode, can::FocMotorState::fault, can::FocFaultCode::overCurrent);

    EXPECT_EQ(can::ContractCompatibility::unknown, bus.Contracts());
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(0.0f, bus.Reading(joint).position.Value(), tolerance);
}

TEST_F(JointBusImplTest, a_joint_entering_fault_is_reported_once_per_episode)
{
    EXPECT_CALL(observer, OnJointFault(2, can::FocFaultCode::overCurrent));
    ReportStatus(nodes[2], can::FocMotorState::fault, can::FocFaultCode::overCurrent);
    ReportStatus(nodes[2], can::FocMotorState::fault, can::FocFaultCode::overCurrent);
    EXPECT_EQ(can::FocFaultCode::overCurrent, bus.Reading(2).fault);

    ReportStatus(nodes[2], can::FocMotorState::idle, can::FocFaultCode::none);
    EXPECT_CALL(observer, OnJointFault(2, can::FocFaultCode::stall));
    ReportStatus(nodes[2], can::FocMotorState::fault, can::FocFaultCode::stall);
}

TEST_F(JointBusImplTest, requests_telemetry_from_every_joint)
{
    EXPECT_TRUE(bus.RequestTelemetry());

    ExpectOneFramePerJoint(can::focRequestTelemetryId);
}

TEST_F(JointBusImplTest, a_joint_missing_the_configured_number_of_requests_in_a_row_is_reported_silent_once)
{
    const auto answerAllBut = [this](std::size_t silentJoint)
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (joint != silentJoint)
                ReportMotion(nodes[joint], 0.0f, 0.0f);
    };

    EXPECT_TRUE(bus.RequestTelemetry());
    for (uint8_t missed = 1; missed != missedRequestsBeforeSilent; ++missed)
    {
        answerAllBut(1);
        EXPECT_TRUE(bus.RequestTelemetry());
    }

    answerAllBut(1);
    EXPECT_CALL(observer, OnJointSilent(1));
    EXPECT_TRUE(bus.RequestTelemetry());

    answerAllBut(1);
    EXPECT_TRUE(bus.RequestTelemetry());
}

TEST_F(JointBusImplTest, a_silent_joint_that_answers_again_is_reported_again_when_it_next_goes_silent)
{
    EXPECT_CALL(observer, OnJointSilent(_)).Times(arm::jointCount);
    for (uint8_t request = 0; request != missedRequestsBeforeSilent + 1; ++request)
        EXPECT_TRUE(bus.RequestTelemetry());

    ReportMotion(nodes[5], 0.0f, 0.0f);
    for (uint8_t request = 0; request != missedRequestsBeforeSilent; ++request)
        EXPECT_TRUE(bus.RequestTelemetry());

    EXPECT_CALL(observer, OnJointSilent(5));
    EXPECT_TRUE(bus.RequestTelemetry());
}

TEST_F(JointBusImplTest, a_silent_joint_reading_is_marked_silent_until_the_joint_answers_again)
{
    EXPECT_CALL(observer, OnJointSilent(_)).Times(arm::jointCount);
    for (uint8_t request = 0; request != missedRequestsBeforeSilent + 1; ++request)
        EXPECT_TRUE(bus.RequestTelemetry());

    EXPECT_TRUE(bus.Reading(2).silent);

    ReportMotion(nodes[2], 0.0f, 0.5f);

    EXPECT_FALSE(bus.Reading(2).silent);
    EXPECT_TRUE(bus.Reading(3).silent);
}

TEST_F(JointBusImplTest, a_command_refused_during_a_telemetry_burst_reaches_every_joint_once_the_burst_drains)
{
    holdTransmissions = true;
    EXPECT_TRUE(bus.RequestTelemetry());

    EXPECT_TRUE(bus.Start());
    EXPECT_LT(FramesTo(nodes.back(), can::focStartId), 1u);

    holdTransmissions = false;
    CompleteTransmissions();
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::start));
    ForwardTime(std::chrono::milliseconds{ 1 });

    for (const auto node : nodes)
        EXPECT_EQ(1u, FramesTo(node, can::focStartId)) << "node " << node;
}

TEST_F(JointBusImplTest, a_held_command_that_cannot_reach_every_joint_in_time_fails)
{
    holdTransmissions = true;
    EXPECT_TRUE(bus.RequestTelemetry());
    EXPECT_TRUE(bus.Start());

    ForwardTime(commandTimeout - std::chrono::milliseconds{ 1 });
    EXPECT_CALL(observer, OnCommandFailed(arm::JointCommand::start, arm::CommandFailure::timedOut));
    ForwardTime(std::chrono::milliseconds{ 1 });

    holdTransmissions = false;
    CompleteTransmissions();
    ForwardTime(commandTimeout);
    EXPECT_EQ(0u, FramesTo(nodes.back(), can::focStartId));
}

TEST_F(JointBusImplTest, a_newer_command_replaces_a_held_one)
{
    holdTransmissions = true;
    EXPECT_TRUE(bus.RequestTelemetry());
    EXPECT_TRUE(bus.Start());

    EXPECT_CALL(observer, OnCommandFailed(arm::JointCommand::start, arm::CommandFailure::replaced));
    EXPECT_TRUE(bus.Stop());

    holdTransmissions = false;
    CompleteTransmissions();
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::stop));
    ForwardTime(std::chrono::milliseconds{ 1 });

    for (const auto node : nodes)
        EXPECT_EQ(1u, FramesTo(node, can::focStopId)) << "node " << node;
    EXPECT_EQ(0u, FramesTo(nodes.back(), can::focStartId));
}

TEST_F(JointBusImplTest, an_emergency_stop_overtakes_a_held_command_and_drops_it)
{
    holdTransmissions = true;
    EXPECT_TRUE(bus.RequestTelemetry());
    EXPECT_TRUE(bus.Start());

    EXPECT_CALL(observer, OnCommandFailed(arm::JointCommand::start, arm::CommandFailure::emergencyStop));
    EXPECT_TRUE(bus.EmergencyStop());

    holdTransmissions = false;
    CompleteTransmissions();
    ForwardTime(commandTimeout);

    for (const auto node : nodes)
        EXPECT_EQ(1u, FramesTo(node, can::focEmergencyStopId)) << "node " << node;
    EXPECT_EQ(0u, FramesTo(nodes.back(), can::focStartId));
}

TEST_F(JointBusImplTest, a_command_to_an_arm_with_an_incompatible_joint_is_refused_before_any_frame_is_sent)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::queryContracts));
    EXPECT_TRUE(bus.QueryContracts());
    EXPECT_CALL(observer, OnContractsResolved(can::ContractCompatibility::incompatible));
    AnswerIncompatible(nodes[4]);
    sent.clear();

    EXPECT_FALSE(bus.Start());

    EXPECT_TRUE(sent.empty());
}
