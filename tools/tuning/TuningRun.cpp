#include "tools/tuning/TuningRun.hpp"
#include "numerical/math/Math.hpp"
#include <utility>

namespace tuning
{
    namespace
    {
        constexpr float maxStep{ 0.2f };

        arm::JointPositions Origin(const arm_host::ArmSnapshot& snapshot)
        {
            if (snapshot.commanded.has_value())
                return *snapshot.commanded;

            arm::JointPositions measured{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured[joint] = snapshot.readings[joint].position;
            return measured;
        }
    }

    std::optional<std::string> Refusal(const TuningTest& test, const arm_host::ArmSnapshot& snapshot)
    {
        if (test.joint >= arm::jointCount)
            return "the joint is 1 to 6";
        if (test.kind == TestKind::step && (test.amplitude == 0.0f || math::Abs(test.amplitude) > maxStep))
            return "a step is not 0 and at most 0.2 rad";
        if (test.amplitude == 0.0f)
            return "the amplitude is not 0";
        if (!snapshot.targetReady)
            return "not connected";
        if (snapshot.state != arm::ArmState::enabled)
            return snapshot.moving ? "the arm is moving" : "the arm is not enabled";
        if (snapshot.moving)
            return "the arm is moving";
        return std::nullopt;
    }

    TuningRun::TuningRun(arm_host::ArmCommands& arm, const TuningTest& test)
        : arm(arm)
        , test(test)
    {}

    std::optional<std::string> TuningRun::Start()
    {
        const auto snapshot = arm.Snapshot();
        if (auto refusal = Refusal(test, snapshot))
            return refusal;

        origin = Origin(snapshot);
        auto target = origin;
        target[test.joint] = foc::Radians{ origin[test.joint].Value() + test.amplitude };
        pending = test.kind == TestKind::step ? arm.Post(arm_host::StepTo{ target }) : arm.Post(arm_host::MoveTo{ target });
        state = RunState::starting;
        return std::nullopt;
    }

    void TuningRun::OnEvent(const arm_host::ArmEvent& event)
    {
        if (state == RunState::idle || state == RunState::done || state == RunState::failed)
            return;

        if (pending != 0 && event.command == pending)
            OnAnswer(event);
        else if (event.kind == arm_host::ArmEventKind::moveAborted)
            Fail("move aborted");
        else if (event.kind == arm_host::ArmEventKind::tripped)
            Fail("tripped: " + event.reason);
        else if (event.kind == arm_host::ArmEventKind::moveDone)
            OnMoveDone();
    }

    void TuningRun::OnAnswer(const arm_host::ArmEvent& event)
    {
        pending = 0;
        if (event.kind == arm_host::ArmEventKind::refused)
            Fail(std::string{ test.kind == TestKind::step ? "step" : "move" } + " refused: " + event.reason);
        else if (event.kind == arm_host::ArmEventKind::accepted)
            state = RunState::recording;
    }

    void TuningRun::OnMoveDone()
    {
        if (test.kind != TestKind::outAndBack || state != RunState::recording)
            return;

        if (back)
        {
            settleStart = samples.empty() ? 0.0f : samples.back().time;
            state = RunState::settling;
            return;
        }

        back = true;
        pending = arm.Post(arm_host::MoveTo{ origin });
        state = RunState::returning;
    }

    void TuningRun::OnTrace(const arm_host::TraceSample& sample)
    {
        if (state == RunState::recording || state == RunState::returning || state == RunState::settling)
            Record(sample);
    }

    void TuningRun::Record(const arm_host::TraceSample& sample)
    {
        if (samples.size() == maxSamples)
        {
            Fail("the run exceeded 2000 samples");
            return;
        }

        if (!firstTime.has_value())
            firstTime = sample.time;
        const auto time = std::chrono::duration<float>(sample.time - *firstTime).count();
        const auto torque = sample.torques.has_value() ? (*sample.torques)[test.joint].Value() : 0.0f;
        samples.push_back({ time, sample.commanded[test.joint].Value(), sample.measured[test.joint].Value(), torque });

        const bool stepRecorded = test.kind == TestKind::step && time >= std::chrono::duration<float>(test.record).count();
        const bool settled = state == RunState::settling && time - settleStart >= settleSeconds;
        if (stepRecorded || settled)
            state = RunState::done;
    }

    void TuningRun::Fail(std::string reason)
    {
        failure = std::move(reason);
        state = RunState::failed;
    }

    uint32_t TuningRun::Command() const
    {
        return pending;
    }

    RunState TuningRun::State() const
    {
        return state;
    }

    const std::string& TuningRun::Failure() const
    {
        return failure;
    }

    const TuningTest& TuningRun::Test() const
    {
        return test;
    }

    const std::vector<TuningSample>& TuningRun::Samples() const
    {
        return samples;
    }

    std::optional<StepMetrics> TuningRun::Step() const
    {
        if (test.kind != TestKind::step || state != RunState::done)
            return std::nullopt;
        return MeasureStep(samples, origin[test.joint].Value(), test.amplitude);
    }

    TrackingMetrics TuningRun::Tracking() const
    {
        return MeasureTracking(samples);
    }
}
