#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_CARTESIAN_IMPEDANCE_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_CARTESIAN_IMPEDANCE_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "core/arm_model/ArmTaskJacobian.hpp"
#include <array>

namespace arm
{
    struct CartesianImpedanceTuning
    {
        float translationalStiffness{ 50.0f };
        float rotationalStiffness{ 0.07f };
        float dampingRatio{ 0.7f };

        bool operator==(const CartesianImpedanceTuning& other) const = default;
    };

    class CartesianImpedance
        : public ArmController
    {
    public:
        CartesianImpedance(const ArmModel& model, const JointInertias& rotors, const CartesianImpedanceTuning& tuning);

        bool SetStiffnessShare(float share);
        float StiffnessShare() const;
        ArmModel::MassMatrix Damping(const ArmModel::JointVector& angles) const;

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        void Size();

        const ArmModel& model;
        ArmTaskJacobian provider;
        JointInertias rotors;
        CartesianImpedanceTuning tuning;
        float share{ 1.0f };
        std::array<float, 6> stiffness{};
    };
}

#endif
