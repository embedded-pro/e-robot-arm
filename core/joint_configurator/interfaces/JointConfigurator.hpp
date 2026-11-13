#ifndef CORE_JOINT_CONFIGURATOR_INTERFACES_JOINT_CONFIGURATOR_HPP
#define CORE_JOINT_CONFIGURATOR_INTERFACES_JOINT_CONFIGURATOR_HPP

#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "infra/util/Observer.hpp"
#include <cstddef>
#include <cstdint>

namespace arm
{
    enum class ConfigurationFailure : uint8_t
    {
        timedOut
    };

    class JointConfigurator;

    class JointConfiguratorObserver
        : public infra::Observer<JointConfiguratorObserver, JointConfigurator>
    {
    public:
        using infra::Observer<JointConfiguratorObserver, JointConfigurator>::Observer;

        virtual void OnApplied(std::size_t joint) = 0;
        virtual void OnRead(std::size_t joint, const ControllerReading& reading) = 0;
        virtual void OnConfigurationFailed(std::size_t joint, ConfigurationFailure failure) = 0;

    protected:
        ~JointConfiguratorObserver() = default;
    };

    class JointConfigurator
        : public infra::Subject<JointConfiguratorObserver>
    {
    public:
        JointConfigurator() = default;
        JointConfigurator(const JointConfigurator& other) = delete;
        JointConfigurator& operator=(const JointConfigurator& other) = delete;

        virtual bool Apply(std::size_t joint, const ControllerSettings& settings) = 0;
        virtual bool Read(std::size_t joint) = 0;
        virtual bool Busy() const = 0;

    protected:
        ~JointConfigurator() = default;
    };
}

#endif
