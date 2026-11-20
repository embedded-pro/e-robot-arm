#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_IMPEDANCE_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_IMPEDANCE_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include <array>
#include <optional>

namespace arm
{
    using JointStiffnesses = std::array<float, jointCount>;

    inline constexpr float defaultImpedanceBandwidth{ 10.0f };
    inline constexpr float minimumStiffnessShare{ 0.1f };

    struct JointImpedanceTuning
    {
        std::optional<JointStiffnesses> stiffness;
        float dampingRatio{ 0.7f };
    };

    JointInertias HomeInertias(const ArmModel& model, const JointInertias& rotors);
    JointStiffnesses ImpedanceStiffness(const JointInertias& inertias, const JointImpedanceTuning& tuning);

    class JointImpedance
        : public ArmController
    {
    public:
        JointImpedance(const ArmModel& model, const JointInertias& inertias, const JointImpedanceTuning& tuning);

        bool SetStiffnessShare(float share);
        float StiffnessShare() const;

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        void Size();

        const ArmModel& model;
        JointInertias inertias;
        JointStiffnesses profileStiffness;
        float dampingRatio;
        float share{ 1.0f };
        JointStiffnesses stiffness{};
        JointStiffnesses damping{};
    };
}

#endif
