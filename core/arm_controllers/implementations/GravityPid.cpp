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

        ArmModel::MassMatrix InertiaAtHome(const ArmModel& model, const JointInertias& rotors)
        {
            auto inertia = model.JointMassMatrix(JointPositions{});
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                inertia.at(joint, joint) += rotors[joint];
            return inertia;
        }
    }

    GravityPid::GravityPid(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning, infra::Duration period)
        : model(model)
        , inertia(InertiaAtHome(model, rotors))
        , derivativeGain(PairGain(tuning) * tuning.bandwidth)
        , pids(MakePids(ProportionalIntegral(tuning, period), std::make_index_sequence<jointCount>{}))
    {}

    bool GravityPid::BumplessTransfer() const
    {
        return true;
    }

    void GravityPid::Reset(const MeasuredJoints&)
    {
        for (auto& pid : pids)
            pid.Reset();
        primed = false;
    }

    JointSetpoints GravityPid::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto corrections = Corrections(measured, reference);
        auto torques = model.GravityTorque(measured.positions);
        for (std::size_t row = 0; row != jointCount; ++row)
            for (std::size_t column = 0; column != jointCount; ++column)
                torques[row] = foc::NewtonMeter{ torques[row].Value() +
                                                 inertia.at(row, column) * (reference.accelerations[column].Value() + corrections[column].Value()) };
        return JointSetpoints{ reference.positions, torques };
    }

    JointAccelerations GravityPid::Corrections(const MeasuredJoints& measured, const JointReference& reference)
    {
        JointAccelerations corrections{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            auto& pid = pids[joint];
            pid.SetPoint(reference.positions[joint].Value());
            if (!primed)
            {
                pid.Process(measured.positions[joint].Value());
                pid.SetPreviousOutput(0.0f);
            }
            const auto damping = derivativeGain * (reference.speeds[joint].Value() - measured.speeds[joint].Value());
            corrections[joint] = foc::RadiansPerSecondSquared{ std::clamp(pid.Process(measured.positions[joint].Value()) + damping, -correctionLimit, correctionLimit) };
        }
        primed = true;
        return corrections;
    }
}
