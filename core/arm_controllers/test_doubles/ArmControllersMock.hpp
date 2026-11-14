#ifndef CORE_ARM_CONTROLLERS_TEST_DOUBLES_ARM_CONTROLLERS_MOCK_HPP
#define CORE_ARM_CONTROLLERS_TEST_DOUBLES_ARM_CONTROLLERS_MOCK_HPP

#include "core/arm_controllers/interfaces/ArmControllers.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class ArmControllersMock
        : public ArmControllers
    {
    public:
        MOCK_METHOD(std::size_t, Count, (), (const, override));
        MOCK_METHOD(infra::BoundedConstString, Name, (ArmControllerId id), (const, override));
        MOCK_METHOD(std::optional<ArmControllerId>, Find, (infra::BoundedConstString name), (const, override));
        MOCK_METHOD(bool, Enabled, (ArmControllerId id), (const, override));
        MOCK_METHOD(std::optional<ArmControllerId>, Active, (), (const, override));
        MOCK_METHOD(bool, Enable, (ArmControllerId id), (override));
        MOCK_METHOD(bool, Disable, (ArmControllerId id), (override));
        MOCK_METHOD(bool, Select, (ArmControllerId id), (override));
    };
}

#endif
