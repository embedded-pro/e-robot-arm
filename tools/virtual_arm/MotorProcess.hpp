#ifndef TOOLS_VIRTUAL_ARM_MOTOR_PROCESS_HPP
#define TOOLS_VIRTUAL_ARM_MOTOR_PROCESS_HPP

#include <filesystem>
#include <optional>
#include <string>
#include <sys/types.h>
#include <vector>

namespace virtual_arm
{
    class MotorProcess
    {
    public:
        MotorProcess() = default;
        MotorProcess(const MotorProcess& other) = delete;
        MotorProcess& operator=(const MotorProcess& other) = delete;
        ~MotorProcess();

        bool Start(const std::filesystem::path& program, const std::vector<std::string>& arguments, const std::filesystem::path& workDirectory,
            const std::filesystem::path& logFile);
        void Stop();
        bool Started() const;
        bool Running() const;
        std::optional<int> Ended();

    private:
        pid_t pid{ -1 };
        std::optional<int> exitCode;
    };
}

#endif
