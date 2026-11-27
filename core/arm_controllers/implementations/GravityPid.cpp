#include "core/arm_controllers/implementations/GravityPid.hpp"
#include <algorithm>
#include <chrono>
#include <utility>

namespace arm
{
    namespace
    {
        float PairGain(const BandwidthTuning& tuning)
        {
            return 2.0f * tuning.dampingRatio + 1.0f;
        }

        controllers::PidTunings<float> ProportionalIntegral(const BandwidthTuning& tuning, infra::Duration period)
        {
            const auto seconds = std::chrono::duration<float>(period).count();
            const auto bandwidth = tuning.bandwidth;
            return { PairGain(tuning) * bandwidth * bandwidth, bandwidth * bandwidth * bandwidth * seconds, 0.0f };
        }

        template<std::size_t... Joint>
        std::array<controllers::PidIncrementalSynchronous<float>, jointCount> MakePids(const controllers::PidTunings<float>& tunings,
            std::index_sequence<Joint...>)
        {
            const controllers::PidLimits<float> limits{ -GravityPid::correctionLimit, GravityPid::correctionLimit };
            return { (static_cast<void>(Joint), controllers::PidIncrementalSynchronous<float>{ tunings, limits })... };
        }
    }

    GravityPid::GravityPid(const DrivenArm& drivenArm, const BandwidthTuning& tuning, infra::Duration period)
        : drivenArm(drivenArm)
        , model(drivenArm.Model())
        , period(period)
        , derivativeGain(PairGain(tuning) * tuning.bandwidth)
        , pids(MakePids(ProportionalIntegral(tuning, period), std::make_index_sequence<jointCount>{}))
    {}

    void GravityPid::SetTuning(const BandwidthTuning& tuning)
    {
        derivativeGain = PairGain(tuning) * tuning.bandwidth;
        for (auto& pid : pids)
            pid.SetTunings(ProportionalIntegral(tuning, period));
    }

    bool GravityPid::BumplessTransfer() const
    {
        return true;
    }

    void GravityPid::Reset(const MeasuredJoints& measured, const JointReference& held)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            auto& pid = pids[joint];
            pid.Reset();
            pid.SetPoint(held.positions[joint].Value());
            pid.Process(measured.positions[joint].Value());
            pid.SetPreviousOutput(0.0f);
        }
    }

    JointSetpoints GravityPid::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const JointVector acceleration{ ToVector(reference.accelerations) + ToVector(Corrections(measured, reference)) };
        const JointVector torques{ ToVector(model.GravityTorque(measured.positions)) + drivenArm.JointMassMatrix(measured.positions) * acceleration };
        return JointSetpoints{ reference.positions, FromVector<foc::NewtonMeter>(torques) };
    }

    JointAccelerations GravityPid::Corrections(const MeasuredJoints& measured, const JointReference& reference)
    {
        JointAccelerations corrections{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            auto& pid = pids[joint];
            pid.SetPoint(reference.positions[joint].Value());
            const auto damping = derivativeGain * (reference.speeds[joint].Value() - measured.speeds[joint].Value());
            corrections[joint] = foc::RadiansPerSecondSquared{ std::clamp(pid.Process(measured.positions[joint].Value()) + damping, -correctionLimit, correctionLimit) };
        }
        return corrections;
    }
}
