#include "core/joint_configurator/test_doubles/JointConfiguratorMock.hpp"
#include "core/lifecycle/implementations/ArmLifecycleImpl.hpp"
#include "core/lifecycle/test_doubles/ArmLifecycleMock.hpp"
#include "core/motion/test_doubles/JointMotionMock.hpp"
#include "core/safety/test_doubles/SafetySupervisorMock.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace testing;
    using arm::ArmState;

    class ArmLifecycleImplTest
        : public Test
    {
    public:
        explicit ArmLifecycleImplTest(const arm::JointControllers& controllers = {}, const arm::JointMotors& motors = {})
            : controllers(controllers)
            , motors(motors)
        {
            EXPECT_CALL(observer, OnStateChanged(_)).Times(AnyNumber());
            EXPECT_CALL(safety, Reading(_)).WillRepeatedly(Invoke([this](std::size_t joint) -> const arm::JointReading&
                {
                    return readings[joint];
                }));
        }

        void SetAll(can::FocMotorState state)
        {
            for (auto& reading : readings)
                reading.state = state;
        }

        void Observe(can::FocMotorState state)
        {
            SetAll(state);
            lifecycle.Step();
        }

        void ResolveContracts(can::ContractCompatibility compatibility)
        {
            safety.NotifyObservers([compatibility](auto& joints)
                {
                    joints.OnContractsResolved(compatibility);
                });
        }

        void Deliver(arm::JointCommand command)
        {
            safety.NotifyObservers([command](auto& joints)
                {
                    joints.OnCommandDelivered(command);
                });
        }

        void Fail(arm::JointCommand command)
        {
            safety.NotifyObservers([command](auto& joints)
                {
                    joints.OnCommandFailed(command, arm::CommandFailure::timedOut);
                });
        }

        void FaultJoint(std::size_t joint)
        {
            safety.NotifyObservers([joint](auto& joints)
                {
                    joints.OnJointFault(joint, can::FocFaultCode::overCurrent);
                });
        }

        void FinishMove()
        {
            motion.NotifyObservers([](auto& moves)
                {
                    moves.OnMoveDone();
                });
        }

        void BringToReady()
        {
            EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
            EXPECT_TRUE(lifecycle.BringUp());
            ResolveContracts(can::ContractCompatibility::compatible);
            Observe(can::FocMotorState::ready);
            ASSERT_EQ(ArmState::ready, lifecycle.State());
        }

        void BringToEnabled()
        {
            BringToReady();
            EXPECT_CALL(safety, SelectMode(can::FocMotorMode::speed)).WillOnce(Return(true));
            EXPECT_TRUE(lifecycle.Enable());
            EXPECT_CALL(safety, Start()).WillOnce(Return(true));
            Deliver(arm::JointCommand::selectMode);
            Observe(can::FocMotorState::running);
            ASSERT_EQ(ArmState::enabled, lifecycle.State());
        }

        std::array<arm::JointReading, arm::jointCount> readings{};
        StrictMock<arm::SafetySupervisorMock> safety;
        StrictMock<arm::JointMotionMock> motion;
        StrictMock<arm::JointConfiguratorMock> configurator;
        arm::JointControllers controllers;
        arm::JointMotors motors;
        arm::ArmLifecycleImpl lifecycle{ safety, motion, configurator, can::FocMotorMode::speed, controllers, motors };
        StrictMock<arm::ArmLifecycleObserverMock> observer{ lifecycle };
        arm::JointPositions target{};
    };
}

TEST_F(ArmLifecycleImplTest, starts_unpowered_and_refuses_everything_but_bring_up)
{
    EXPECT_EQ(ArmState::unpowered, lifecycle.State());
    EXPECT_FALSE(lifecycle.Enable());
    EXPECT_FALSE(lifecycle.Disable());
    EXPECT_FALSE(lifecycle.Rearm());
    EXPECT_FALSE(lifecycle.MoveTo(target));
    EXPECT_EQ(ArmState::unpowered, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, bring_up_queries_the_contracts_and_reports_each_state_change)
{
    InSequence sequence;
    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_CALL(observer, OnStateChanged(ArmState::checking));
    EXPECT_TRUE(lifecycle.BringUp());

    EXPECT_CALL(observer, OnStateChanged(ArmState::waitingForCalibration));
    ResolveContracts(can::ContractCompatibility::compatible);
}

TEST_F(ArmLifecycleImplTest, waits_until_every_joint_is_calibrated)
{
    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());
    ResolveContracts(can::ContractCompatibility::compatible);

    SetAll(can::FocMotorState::ready);
    readings[4].state = can::FocMotorState::partialCalibration;
    lifecycle.Step();
    EXPECT_EQ(ArmState::waitingForCalibration, lifecycle.State());

    readings[4].state = can::FocMotorState::ready;
    lifecycle.Step();
    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, incompatible_contracts_stop_the_bring_up_until_it_is_tried_again)
{
    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());
    ResolveContracts(can::ContractCompatibility::incompatible);
    EXPECT_EQ(ArmState::incompatible, lifecycle.State());

    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());
    EXPECT_EQ(ArmState::checking, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_contract_query_that_is_not_delivered_returns_to_unpowered)
{
    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());

    Fail(arm::JointCommand::queryContracts);

    EXPECT_EQ(ArmState::unpowered, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_contract_query_directly_on_the_joints_also_brings_the_arm_up)
{
    ResolveContracts(can::ContractCompatibility::compatible);

    EXPECT_EQ(ArmState::waitingForCalibration, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, enable_selects_the_configured_mode_then_starts_the_joints)
{
    BringToReady();

    EXPECT_CALL(safety, SelectMode(can::FocMotorMode::speed)).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.Enable());
    EXPECT_EQ(ArmState::enabling, lifecycle.State());

    EXPECT_CALL(safety, Start()).WillOnce(Return(true));
    Deliver(arm::JointCommand::selectMode);
    Observe(can::FocMotorState::ready);
    EXPECT_EQ(ArmState::enabling, lifecycle.State());

    Observe(can::FocMotorState::running);
    EXPECT_EQ(ArmState::enabled, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, enable_falls_back_to_ready_when_the_start_is_not_delivered)
{
    BringToReady();
    EXPECT_CALL(safety, SelectMode(can::FocMotorMode::speed)).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.Enable());
    EXPECT_CALL(safety, Start()).WillOnce(Return(true));
    Deliver(arm::JointCommand::selectMode);

    Fail(arm::JointCommand::start);

    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, enable_refused_by_the_safety_supervisor_falls_back_to_ready)
{
    BringToReady();
    EXPECT_CALL(safety, SelectMode(can::FocMotorMode::speed)).WillOnce(Return(false));

    EXPECT_TRUE(lifecycle.Enable());

    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, joints_started_directly_bring_the_arm_to_enabled)
{
    BringToReady();

    Observe(can::FocMotorState::running);

    EXPECT_EQ(ArmState::enabled, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_move_starts_only_while_enabled_and_ends_back_in_enabled)
{
    BringToEnabled();

    EXPECT_CALL(motion, MoveTo(_)).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.MoveTo(target));
    EXPECT_EQ(ArmState::moving, lifecycle.State());
    EXPECT_FALSE(lifecycle.MoveTo(target));

    FinishMove();
    EXPECT_EQ(ArmState::enabled, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_move_refused_by_joint_motion_leaves_the_arm_enabled)
{
    BringToEnabled();
    EXPECT_CALL(motion, MoveTo(_)).WillOnce(Return(false));

    EXPECT_FALSE(lifecycle.MoveTo(target));

    EXPECT_EQ(ArmState::enabled, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_joint_that_stops_running_aborts_the_move_and_falls_back_to_ready)
{
    BringToEnabled();
    EXPECT_CALL(motion, MoveTo(_)).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.MoveTo(target));

    EXPECT_CALL(motion, Abort());
    readings[3].state = can::FocMotorState::ready;
    lifecycle.Step();

    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, disable_stops_the_joints_and_returns_to_ready_once_none_runs)
{
    BringToEnabled();

    EXPECT_CALL(safety, Stop()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.Disable());
    EXPECT_EQ(ArmState::disabling, lifecycle.State());

    readings[0].state = can::FocMotorState::ready;
    lifecycle.Step();
    EXPECT_EQ(ArmState::disabling, lifecycle.State());

    Observe(can::FocMotorState::ready);
    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_stop_that_is_not_delivered_while_disabling_emergency_stops_and_trips)
{
    BringToEnabled();
    EXPECT_CALL(safety, Stop()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.Disable());

    EXPECT_CALL(safety, EmergencyStop()).WillOnce(Return(true));
    Fail(arm::JointCommand::stop);

    EXPECT_EQ(ArmState::tripped, lifecycle.State());
}

TEST_F(ArmLifecycleImplTest, a_fault_trips_a_moving_arm_and_aborts_the_move)
{
    BringToEnabled();
    EXPECT_CALL(motion, MoveTo(_)).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.MoveTo(target));

    EXPECT_CALL(motion, Abort());
    FaultJoint(2);

    EXPECT_EQ(ArmState::tripped, lifecycle.State());
    EXPECT_FALSE(lifecycle.Enable());
    EXPECT_FALSE(lifecycle.BringUp());
}

TEST_F(ArmLifecycleImplTest, rearm_needs_the_safety_supervisor_and_then_waits_for_calibration_again)
{
    BringToReady();
    FaultJoint(2);
    EXPECT_EQ(ArmState::tripped, lifecycle.State());

    EXPECT_CALL(safety, Rearm()).WillOnce(Return(false));
    EXPECT_FALSE(lifecycle.Rearm());
    EXPECT_EQ(ArmState::tripped, lifecycle.State());

    EXPECT_CALL(safety, Rearm()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.Rearm());
    EXPECT_EQ(ArmState::waitingForCalibration, lifecycle.State());

    Observe(can::FocMotorState::ready);
    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

namespace
{
    arm::ControllerSettings Settings(float speedBandwidth)
    {
        arm::ControllerSettings settings{};
        settings.position = can::FocPositionAlgorithm::lqr;
        settings[arm::Gain::speedBandwidth] = speedBandwidth;
        return settings;
    }

    arm::JointControllers TwoConfiguredJoints()
    {
        arm::JointControllers controllers{};
        controllers[1] = Settings(100.0f);
        controllers[4] = Settings(140.0f);
        return controllers;
    }

    class ArmLifecycleConfiguringTest
        : public ArmLifecycleImplTest
    {
    public:
        ArmLifecycleConfiguringTest()
            : ArmLifecycleImplTest(TwoConfiguredJoints())
        {
            EXPECT_CALL(configurator, Busy()).WillRepeatedly(Return(false));
        }

        void BringToConfiguring()
        {
            EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
            EXPECT_TRUE(lifecycle.BringUp());
            ResolveContracts(can::ContractCompatibility::compatible);
            Observe(can::FocMotorState::ready);
            ASSERT_EQ(ArmState::configuring, lifecycle.State());
        }

        void Applied(std::size_t joint)
        {
            configurator.NotifyObservers([joint](auto& observer)
                {
                    observer.OnApplied(joint);
                });
        }

        void ReadBack(std::size_t joint, const arm::ControllerSettings& settings)
        {
            configurator.NotifyObservers([joint, &settings](auto& observer)
                {
                    observer.OnRead(joint, arm::ControllerReading{ can::FocMotorMode::position, settings });
                });
        }

        void ConfigureJoint(std::size_t joint)
        {
            EXPECT_CALL(configurator, Read(joint)).WillOnce(Return(true));
            Applied(joint);
        }
    };
}

TEST_F(ArmLifecycleConfiguringTest, after_calibration_applies_and_verifies_each_configured_joint_before_it_is_ready)
{
    BringToConfiguring();

    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    lifecycle.Step();
    ConfigureJoint(1);

    EXPECT_CALL(configurator, Apply(4, _)).WillOnce(Invoke([](std::size_t, const arm::ControllerSettings& settings)
        {
            EXPECT_NEAR(140.0f, settings[arm::Gain::speedBandwidth], 1.0e-6f);
            return true;
        }));
    ReadBack(1, Settings(100.0f));
    EXPECT_EQ(ArmState::configuring, lifecycle.State());
    EXPECT_FALSE(lifecycle.Enable());

    ConfigureJoint(4);
    EXPECT_CALL(observer, OnStateChanged(ArmState::ready));
    ReadBack(4, Settings(140.0f));
    EXPECT_EQ(ArmState::ready, lifecycle.State());
}

TEST_F(ArmLifecycleConfiguringTest, a_read_back_that_differs_stops_the_bring_up_and_names_the_joint)
{
    BringToConfiguring();
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    lifecycle.Step();
    ConfigureJoint(1);

    ReadBack(1, Settings(90.0f));
    EXPECT_EQ(ArmState::misconfigured, lifecycle.State());
    EXPECT_EQ((arm::Misconfiguration{ 1, arm::MisconfigurationReason::differs }), lifecycle.MisconfiguredJoint());
    lifecycle.Step();
    EXPECT_FALSE(lifecycle.Enable());

    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());
    EXPECT_EQ(ArmState::checking, lifecycle.State());
    EXPECT_EQ(std::nullopt, lifecycle.MisconfiguredJoint());
}

TEST_F(ArmLifecycleConfiguringTest, an_apply_that_fails_or_is_refused_stops_the_bring_up)
{
    BringToConfiguring();
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    lifecycle.Step();
    configurator.NotifyObservers([](auto& observer)
        {
            observer.OnConfigurationFailed(1, arm::ConfigurationFailure::timedOut);
        });
    EXPECT_EQ((arm::Misconfiguration{ 1, arm::MisconfigurationReason::failed }), lifecycle.MisconfiguredJoint());

    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_TRUE(lifecycle.BringUp());
    ResolveContracts(can::ContractCompatibility::compatible);
    Observe(can::FocMotorState::ready);
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(false));
    lifecycle.Step();
    EXPECT_EQ((arm::Misconfiguration{ 1, arm::MisconfigurationReason::failed }), lifecycle.MisconfiguredJoint());
}

TEST_F(ArmLifecycleConfiguringTest, losing_calibration_waits_again_and_ignores_the_late_outcome)
{
    BringToConfiguring();
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    lifecycle.Step();

    Observe(can::FocMotorState::idle);
    EXPECT_EQ(ArmState::waitingForCalibration, lifecycle.State());
    Applied(1);
    ReadBack(1, Settings(90.0f));
    EXPECT_EQ(ArmState::waitingForCalibration, lifecycle.State());

    Observe(can::FocMotorState::ready);
    EXPECT_EQ(ArmState::configuring, lifecycle.State());
    Applied(1);
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    lifecycle.Step();
}

namespace
{
    arm::JointMotors MotorOnJointOne()
    {
        arm::JointMotors motors{};
        motors[1] = arm::JointMotor{ 0, arm::motorPresets[0].settings };
        return motors;
    }

    class ArmLifecycleMotorFirstTest
        : public ArmLifecycleImplTest
    {
    public:
        ArmLifecycleMotorFirstTest()
            : ArmLifecycleImplTest(TwoConfiguredJoints(), MotorOnJointOne())
        {
            EXPECT_CALL(configurator, Busy()).WillRepeatedly(Return(false));
            EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
            EXPECT_TRUE(lifecycle.BringUp());
            ResolveContracts(can::ContractCompatibility::compatible);
            Observe(can::FocMotorState::ready);
        }

        void MotorAnswers(std::size_t joint, const arm::MotorSettings& motor)
        {
            configurator.NotifyObservers([joint](auto& observer)
                {
                    observer.OnMotorApplied(joint);
                });
            configurator.NotifyObservers([joint, &motor](auto& observer)
                {
                    observer.OnMotorRead(joint, motor);
                });
        }
    };
}

TEST_F(ArmLifecycleMotorFirstTest, configures_a_joints_motor_before_its_motor_controller)
{
    InSequence order;
    EXPECT_CALL(configurator, ApplyMotor(1, _)).WillOnce(Return(true));
    lifecycle.Step();

    EXPECT_CALL(configurator, ReadMotor(1)).WillOnce(Return(true));
    EXPECT_CALL(configurator, Apply(1, _)).WillOnce(Return(true));
    MotorAnswers(1, arm::motorPresets[0].settings);
    EXPECT_EQ(ArmState::configuring, lifecycle.State());
}

TEST_F(ArmLifecycleMotorFirstTest, a_motor_that_reads_back_different_names_the_joint_and_the_motor)
{
    EXPECT_CALL(configurator, ApplyMotor(1, _)).WillOnce(Return(true));
    lifecycle.Step();

    EXPECT_CALL(configurator, ReadMotor(1)).WillOnce(Return(true));
    MotorAnswers(1, arm::motorPresets[1].settings);

    EXPECT_EQ((arm::Misconfiguration{ 1, arm::MisconfigurationReason::differs, arm::ConfigurationPart::motor }), lifecycle.MisconfiguredJoint());
}
