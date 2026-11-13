#ifndef CORE_JOINT_CONFIGURATOR_TEST_DOUBLES_JOINT_CONFIGURATOR_MOCK_HPP
#define CORE_JOINT_CONFIGURATOR_TEST_DOUBLES_JOINT_CONFIGURATOR_MOCK_HPP

#include "core/joint_configurator/interfaces/JointConfigurator.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class JointConfiguratorObserverMock
        : public JointConfiguratorObserver
    {
    public:
        using JointConfiguratorObserver::JointConfiguratorObserver;

        MOCK_METHOD(void, OnApplied, (std::size_t joint), (override));
        MOCK_METHOD(void, OnRead, (std::size_t joint, const ControllerReading& reading), (override));
        MOCK_METHOD(void, OnConfigurationFailed, (std::size_t joint, ConfigurationFailure failure), (override));
    };
}

#endif
