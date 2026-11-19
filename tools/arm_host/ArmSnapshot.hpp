#ifndef TOOLS_ARM_HOST_ARM_SNAPSHOT_HPP
#define TOOLS_ARM_HOST_ARM_SNAPSHOT_HPP

#include "core/arm/ArmConfig.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmTarget.hpp"
#include "tools/arm_host/VirtualMotors.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace arm_host
{
    struct ArmControllerStatus
    {
        std::string name;
        bool enabled{ false };
        bool active{ false };
    };

    struct Disturbances
    {
        bool injectFault{ false };
        bool collision{ false };
        bool powerOff{ false };
        std::array<bool, arm::jointCount> silent{};
        arm::JointTorques torque{};
    };

    enum class StopCategory : uint8_t
    {
        protective,
        emergency
    };

    struct StopReport
    {
        StopCategory category{ StopCategory::protective };
        std::size_t joint{ noJoint };
        std::string reason{};
        can::FocFaultCode fault{ can::FocFaultCode::none };
    };

    struct ArmSnapshot
    {
        uint64_t sequence{ 0 };
        ArmTarget target{};
        bool targetReady{ false };
        std::array<arm::JointReading, arm::jointCount> readings{};
        std::optional<arm::JointPositions> commanded{};
        can::ContractCompatibility contracts{ can::ContractCompatibility::unknown };
        bool tripped{ false };
        bool holding{ false };
        bool hardwareStopOpen{ false };
        arm::JointFlags torqueClamped{};
        std::optional<StopReport> stop{};
        bool moving{ false };
        arm::ArmState state{ arm::ArmState::unpowered };
        std::optional<arm::Misconfiguration> misconfiguration{};
        uint32_t overruns{ 0 };
        bool commissioning{ false };
        arm::Placement toolPose{};
        arm::Singularities singularities{};
        arm::JointTorques gravityTorque{};
        std::optional<arm::JointTorques> externalTorques{};
        float manipulability{ 0.0f };
        std::string profileName{};
        arm::ArmConfig config{};
        std::vector<ArmControllerStatus> armControllers{};
        std::optional<VirtualMotorsStatus> virtualMotors{};
        std::optional<Disturbances> disturbances{};
    };

    struct TraceSample
    {
        infra::TimePoint time{};
        arm::JointPositions commanded{};
        arm::JointPositions measured{};
        std::optional<arm::JointTorques> externalTorques{};
        std::optional<arm::JointTorques> torques{};
    };

    class ArmSessionSink
    {
    public:
        virtual void OnEvent(const ArmEvent& event) = 0;
        virtual void OnSnapshot(const ArmSnapshot& snapshot) = 0;
        virtual void OnTrace(const TraceSample& sample) = 0;

    protected:
        ~ArmSessionSink() = default;
    };
}

#endif
