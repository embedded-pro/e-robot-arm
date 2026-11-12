#ifndef TOOLS_ARM_CONSOLE_ARM_CONSOLE_HPP
#define TOOLS_ARM_CONSOLE_ARM_CONSOLE_HPP

#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace arm_console
{
    class ArmConsole
        : private arm::JointBusObserver
        , private arm::JointMotionObserver
    {
    public:
        ArmConsole(arm::SafetySupervisor& safety, arm::JointMotion& motion, std::ostream& output);

        bool Execute(const std::string& line);

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnMoveDone() override;
        void OnMoveAborted() override;

        bool Dispatch(std::string_view command, std::istringstream& arguments);
        bool Mode(std::istringstream& arguments);
        bool Move(std::istringstream& arguments);
        void Status();
        void Answer(bool accepted);
        void Usage();

        arm::SafetySupervisor& safety;
        arm::JointMotion& motion;
        std::ostream& output;
    };
}

#endif
