#ifndef TOOLS_SIMULATOR_VIEW_MODELS_TEST_SNAPSHOT_BUILDER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_TEST_SNAPSHOT_BUILDER_HPP

#include "tools/arm_host/ArmSnapshot.hpp"

namespace simulator::test
{
    inline arm_host::ArmSnapshot Connected()
    {
        arm_host::ArmSnapshot snapshot{};
        snapshot.targetReady = true;
        return snapshot;
    }

    inline arm_host::ArmSnapshot Compatible()
    {
        auto snapshot = Connected();
        snapshot.contracts = can::ContractCompatibility::compatible;
        return snapshot;
    }

    inline arm_host::ArmSnapshot Running()
    {
        auto snapshot = Compatible();
        for (auto& reading : snapshot.readings)
            reading.state = can::FocMotorState::running;
        return snapshot;
    }

    inline arm::JointLimits Limits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } };
        return limits;
    }
}

#endif
