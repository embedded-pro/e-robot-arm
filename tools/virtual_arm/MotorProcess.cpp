#include "tools/virtual_arm/MotorProcess.hpp"
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace virtual_arm
{
    namespace
    {
        constexpr int execFailed{ 127 };

        int ExitCodeOf(int status)
        {
            return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        }
    }

    MotorProcess::~MotorProcess()
    {
        Stop();
    }

    bool MotorProcess::Start(const std::filesystem::path& program, const std::vector<std::string>& arguments, const std::filesystem::path& workDirectory,
        const std::filesystem::path& logFile)
    {
        std::vector command{ program.string() };
        command.insert(command.end(), arguments.begin(), arguments.end());
        std::vector<char*> argv;
        for (auto& argument : command)
            argv.push_back(argument.data());
        argv.push_back(nullptr);

        pid = fork();
        if (pid == 0)
        {
            sigset_t none;
            sigemptyset(&none);
            sigprocmask(SIG_SETMASK, &none, nullptr);
            setpgid(0, 0);
            prctl(PR_SET_PDEATHSIG, SIGTERM);
            if (const auto log = open(logFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600); log >= 0)
            {
                dup2(log, STDOUT_FILENO);
                dup2(log, STDERR_FILENO);
                close(log);
            }
            if (chdir(workDirectory.c_str()) == 0)
                execvp(argv[0], argv.data());
            _exit(execFailed);
        }

        exitCode.reset();
        return pid > 0;
    }

    void MotorProcess::Stop()
    {
        if (!Running())
            return;

        kill(pid, SIGTERM);
        for (int attempt = 0; attempt != 100 && !Ended().has_value(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });

        if (Running())
        {
            kill(pid, SIGKILL);
            int status{ 0 };
            waitpid(pid, &status, 0);
            exitCode = ExitCodeOf(status);
        }
        pid = -1;
    }

    bool MotorProcess::Running() const
    {
        return pid > 0 && !exitCode.has_value();
    }

    std::optional<int> MotorProcess::Ended()
    {
        if (pid > 0 && !exitCode.has_value())
        {
            int status{ 0 };
            if (waitpid(pid, &status, WNOHANG) == pid)
                exitCode = ExitCodeOf(status);
        }
        return exitCode;
    }
}
