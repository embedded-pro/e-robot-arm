#ifndef CORE_JOINT_CONFIGURATOR_TEST_DOUBLES_JOINT_CONFIGURATOR_MOCK_HPP
#define CORE_JOINT_CONFIGURATOR_TEST_DOUBLES_JOINT_CONFIGURATOR_MOCK_HPP

#include "core/joint_configurator/interfaces/JointConfigurator.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class JointConfiguratorMock
        : public JointConfigurator
    {
    public:
        MOCK_METHOD(bool, Apply, (std::size_t joint, const ControllerSettings& settings), (override));
        MOCK_METHOD(bool, Read, (std::size_t joint), (override));
        MOCK_METHOD(bool, ApplyMotor, (std::size_t joint, const MotorSettings& motor), (override));
        MOCK_METHOD(bool, ReadMotor, (std::size_t joint), (override));
        MOCK_METHOD(bool, Busy, (), (const, override));
    };

    class JointConfiguratorObserverMock
        : public JointConfiguratorObserver
    {
    public:
        using JointConfiguratorObserver::JointConfiguratorObserver;

        MOCK_METHOD(void, OnApplied, (std::size_t joint), (override));
        MOCK_METHOD(void, OnRead, (std::size_t joint, const ControllerReading& reading), (override));
        MOCK_METHOD(void, OnMotorApplied, (std::size_t joint), (override));
        MOCK_METHOD(void, OnMotorRead, (std::size_t joint, const MotorSettings& motor), (override));
        MOCK_METHOD(void, OnConfigurationFailed, (std::size_t joint, ConfigurationFailure failure), (override));
    };
}

#endif
