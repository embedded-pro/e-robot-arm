#include "tools/virtual_arm/VirtualArm.hpp"
#include "can-lite/drivers/implementation/LoopbackSocket.hpp"
#include <algorithm>
#include <format>
#include <poll.h>

namespace virtual_arm
{
    namespace
    {
        constexpr auto couplingPeriod{ std::chrono::milliseconds{ 1 } };

        std::string LastLines(const plant_port::MotorConnection& connection)
        {
            std::string lines;
            for (const auto& line : connection.RecentLog())
                lines += "\n  " + line;
            return lines;
        }

        bool HubAnswers(uint16_t port)
        {
            const auto probe = services::loopback::Connect(port);
            services::loopback::Close(probe);
            return probe != services::loopback::invalidSocket;
        }

        bool HubPortUsable(uint16_t port)
        {
            auto socket = services::loopback::Listen(port);
            if (socket == services::loopback::invalidSocket)
                socket = services::loopback::Connect(port);
            services::loopback::Close(socket);
            return socket != services::loopback::invalidSocket;
        }

        bool WaitForHubPort(uint16_t port, std::chrono::milliseconds timeout)
        {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (!HubPortUsable(port))
            {
                if (std::chrono::steady_clock::now() >= deadline)
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds{ 250 });
            }
            return true;
        }

        MotorPhase PhaseOf(MotorProcess& process, bool joined)
        {
            if (!process.Started())
                return MotorPhase::stopped;
            if (process.Ended().has_value())
                return MotorPhase::ended;
            return joined ? MotorPhase::up : MotorPhase::starting;
        }
    }

    VirtualArm::VirtualArm(const arm::ArmConfig& config, VirtualArmOptions options)
        : config(config)
        , options(std::move(options))
    {
        coupling.emplace(config);
        if (this->options.engine.has_parent_path())
            this->options.engine = std::filesystem::absolute(this->options.engine);
    }

    VirtualArm::~VirtualArm()
    {
        Stop();
    }

    std::optional<std::string> VirtualArm::Start()
    {
        const auto planned = PlanVirtualArm(config);
        if (const auto* refusal = std::get_if<std::string>(&planned))
            return *refusal;
        auto plan = std::get<VirtualArmPlan>(planned);
        for (auto& motor : plan)
            motor.hostsBus = motor.hostsBus && options.hostBus;

        std::error_code error;
        std::filesystem::create_directories(options.workDirectory, error);
        if (error)
            return std::format("cannot create {}: {}", options.workDirectory.string(), error.message());

        if (!options.hostBus && !HubAnswers(options.hubPort))
            return std::format("no virtual CAN hub listens on port {}", options.hubPort);
        if (options.hostBus && !WaitForHubPort(options.hubPort, options.hubPortTimeout))
            return std::format("the virtual CAN hub port {} is still held by a closed connection", options.hubPort);

        listener.emplace();
        if (!listener->Port().has_value())
            return std::string{ "cannot listen on a loopback port for the plant ports" };

        if (auto refusal = StartMotors(plan); refusal.has_value())
        {
            Stop();
            return refusal;
        }

        worker = std::jthread{ [this](const std::stop_token& stop)
            {
                Run(stop);
            } };

        auto failure = WaitUntilUp();
        if (!failure.has_value())
            return std::nullopt;

        Stop();
        if (failure->empty())
        {
            failure = std::format("not every motor came up within {} s:", std::chrono::duration<double>(options.upTimeout).count());
            for (const auto& motor : motors)
                if (!motor.connection.Introduction().has_value())
                    *failure += std::format(" joint {}", motor.start.joint + 1);
        }
        return failure;
    }

    std::optional<std::string> VirtualArm::StartMotors(const VirtualArmPlan& plan)
    {
        const auto host = std::ranges::find_if(plan, [](const MotorStart& motor)
            {
                return motor.hostsBus;
            });
        if (host != plan.end())
        {
            if (auto refusal = StartMotor(*host); refusal.has_value())
                return refusal;
            if (auto refusal = WaitForHub(motors[host->joint]); refusal.has_value())
                return refusal;
        }

        for (const auto& motor : plan)
            if (!motor.hostsBus)
                if (auto refusal = StartMotor(motor); refusal.has_value())
                    return refusal;
        return std::nullopt;
    }

    std::optional<std::string> VirtualArm::StartMotor(const MotorStart& start)
    {
        auto& motor = motors[start.joint];
        motor.start = start;
        const auto log = std::filesystem::absolute(options.workDirectory / std::format("joint{}.log", start.joint + 1));
        if (!motor.process.Start(options.engine, EngineArguments(start, options.bus, *listener->Port(), options.hubPort), options.workDirectory, log))
            return std::format("cannot start {}", options.engine.string());
        return std::nullopt;
    }

    std::optional<std::string> VirtualArm::WaitForHub(Motor& host)
    {
        const auto deadline = std::chrono::steady_clock::now() + options.upTimeout;
        while (true)
        {
            if (HubAnswers(options.hubPort))
                return std::nullopt;
            if (const auto exitCode = host.process.Ended(); exitCode.has_value())
                return std::format("joint {}'s motor ended before it came up, exit code {}", host.start.joint + 1, *exitCode);
            if (std::chrono::steady_clock::now() >= deadline)
                return std::format("joint {}'s motor did not open the virtual CAN hub within {} s", host.start.joint + 1,
                    std::chrono::duration<double>(options.upTimeout).count());
            std::this_thread::sleep_for(std::chrono::milliseconds{ 20 });
        }
    }

    std::optional<std::string> VirtualArm::WaitUntilUp()
    {
        std::unique_lock lock{ mutex };
        changed.wait_for(lock, options.upTimeout, [this]()
            {
                return up || problem.has_value();
            });
        if (up)
            return std::nullopt;
        return problem.value_or(std::string{});
    }

    void VirtualArm::Stop()
    {
        if (worker.joinable())
        {
            worker.request_stop();
            worker.join();
        }

        for (auto motor = motors.rbegin(); motor != motors.rend(); ++motor)
        {
            motor->socket.reset();
            motor->process.Stop();
        }
        arrivals.clear();
        listener.reset();
        Publish();

        std::scoped_lock lock{ mutex };
        up = false;
    }

    bool VirtualArm::Up() const
    {
        std::scoped_lock lock{ mutex };
        return up;
    }

    const std::string& VirtualArm::Bus() const
    {
        return options.bus;
    }

    std::optional<std::string> VirtualArm::Problem() const
    {
        std::scoped_lock lock{ mutex };
        return problem;
    }

    MotorReports VirtualArm::Motors() const
    {
        std::scoped_lock lock{ mutex };
        return reports;
    }

    void VirtualArm::Silence(std::size_t joint, bool silent)
    {
        Post([this, joint, silent]()
            {
                if (joint >= arm::jointCount)
                    return;
                if (silent)
                    motors[joint].connection.Pause();
                else
                    motors[joint].connection.Resume();
            });
    }

    void VirtualArm::Collide(std::size_t joint, foc::NewtonMeter torque)
    {
        Post([this, joint, torque]()
            {
                if (joint < arm::jointCount)
                    collision[joint] = torque;
            });
    }

    std::optional<std::string> VirtualArm::Restart(std::size_t joint, const std::optional<arm::JointMotor>& motor)
    {
        if (auto refusal = RefusalToCycle(joint); refusal.has_value())
            return refusal;

        auto plant = PlantOf(motor);
        if (!plant.has_value())
            return std::format("joint {} has a custom motor: a virtual motor takes e-foc's plant presets only", joint + 1);

        Post([this, joint, plant = std::move(*plant)]()
            {
                RestartMotor(joint, plant);
            });
        return std::nullopt;
    }

    std::optional<std::string> VirtualArm::PowerOff(std::size_t joint)
    {
        if (auto refusal = RefusalToCycle(joint); refusal.has_value())
            return refusal;

        Post([this, joint]()
            {
                PowerOffMotor(joint);
            });
        return std::nullopt;
    }

    std::optional<std::string> VirtualArm::Adopt(const arm::ArmConfig& newConfig)
    {
        if (!Up())
            return std::string{ "the virtual arm is not up" };
        if (newConfig.nodes != config.nodes)
            return std::string{ "the profile's node ids changed: restart the whole virtual arm" };

        const auto running = Motors();
        std::array<std::optional<std::string>, arm::jointCount> changed{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            auto plant = PlantOf(newConfig.motors[joint]);
            if (!plant.has_value())
                return std::format("joint {} has a custom motor: a virtual motor takes e-foc's plant presets only", joint + 1);
            if (*plant == running[joint].plant)
                continue;
            if (motors[joint].start.hostsBus)
                return std::format("joint {}'s motor hosts the virtual CAN bus: restart the whole virtual arm", joint + 1);
            changed[joint] = std::move(*plant);
        }

        config = newConfig;
        Post([this, newConfig, changed]()
            {
                coupling.emplace(newConfig);
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                    if (changed[joint].has_value())
                        RestartMotor(joint, *changed[joint]);
            });
        return std::nullopt;
    }

    std::optional<std::string> VirtualArm::RefusalToCycle(std::size_t joint) const
    {
        if (!Up())
            return std::string{ "the virtual arm is not up" };
        if (joint >= arm::jointCount)
            return std::format("no joint {}", joint + 1);
        if (motors[joint].start.hostsBus)
            return std::format("joint {}'s motor hosts the virtual CAN bus: restart the whole virtual arm", joint + 1);
        return std::nullopt;
    }

    void VirtualArm::RestartMotor(std::size_t joint, std::string_view plant)
    {
        PowerOffMotor(joint);

        auto& motor = motors[joint];
        if (motor.start.plant != plant)
        {
            std::error_code ignored;
            std::filesystem::remove(options.workDirectory / motor.start.memoryFile, ignored);
            motor.start.plant = plant;
        }

        if (auto refusal = StartMotor(motor.start); refusal.has_value())
            Report(*refusal);
    }

    void VirtualArm::PowerOffMotor(std::size_t joint)
    {
        auto& motor = motors[joint];
        motor.socket.reset();
        motor.connection = plant_port::MotorConnection{};
        motor.process.Stop();
    }

    void VirtualArm::Run(const std::stop_token& stop)
    {
        nextCoupling = std::chrono::steady_clock::now();
        while (!stop.stop_requested())
        {
            Poll();
            RunRequests();
            Receive();
            Identify();
            Watch();
            Couple();
            Send();
            Publish();
        }
    }

    void VirtualArm::Poll()
    {
        std::vector<pollfd> descriptors{ { listener->Descriptor(), POLLIN, 0 } };
        for (const auto& arrival : arrivals)
            descriptors.push_back({ arrival.socket.Descriptor(), POLLIN, 0 });
        for (const auto& motor : motors)
            if (motor.socket.has_value())
                descriptors.push_back({ motor.socket->Descriptor(), POLLIN, 0 });

        const auto wait = std::clamp(std::chrono::duration_cast<std::chrono::nanoseconds>(nextCoupling - std::chrono::steady_clock::now()),
            std::chrono::nanoseconds::zero(), std::chrono::nanoseconds{ couplingPeriod });
        const timespec timeout{ 0, static_cast<long>(wait.count()) };
        ppoll(descriptors.data(), descriptors.size(), &timeout, nullptr);

        while (auto socket = listener->Accept())
            arrivals.push_back({ std::move(*socket), {} });
    }

    void VirtualArm::Receive()
    {
        for (auto& arrival : arrivals)
            arrival.connection.Receive(arrival.socket.Read());
        for (auto& motor : motors)
            if (motor.socket.has_value())
                motor.connection.Receive(motor.socket->Read());
    }

    void VirtualArm::Identify()
    {
        for (auto arrival = arrivals.begin(); arrival != arrivals.end();)
        {
            if (const auto& failure = arrival->connection.Failure())
            {
                Report("a motor's plant port failed: " + *failure);
                arrival = arrivals.erase(arrival);
                continue;
            }

            const auto& hello = arrival->connection.Introduction();
            const auto motor = hello.has_value() ? std::ranges::find(motors, hello->node, [](const Motor& candidate)
                                                       {
                                                           return candidate.start.boardId;
                                                       })
                                                 : motors.end();
            if (motor == motors.end())
            {
                ++arrival;
                continue;
            }

            motor->socket = std::move(arrival->socket);
            motor->connection = std::move(arrival->connection);
            motor->connection.Stream(true);
            arrival = arrivals.erase(arrival);
        }
    }

    void VirtualArm::Watch()
    {
        for (auto& motor : motors)
        {
            const auto exitCode = motor.process.Started() ? motor.process.Ended() : std::nullopt;
            if (!exitCode.has_value())
                continue;

            if (motor.socket.has_value())
            {
                Report(std::format("joint {}'s motor ended unasked, exit code {}{}", motor.start.joint + 1, *exitCode, LastLines(motor.connection)));
                motor.socket.reset();
            }
            else if (!motor.connection.Introduction().has_value())
                Report(std::format("joint {}'s motor ended before it came up, exit code {}", motor.start.joint + 1, *exitCode));
        }
    }

    void VirtualArm::Couple()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now < nextCoupling)
            return;
        nextCoupling = std::max(nextCoupling + couplingPeriod, now);

        MotorStates states{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto angle = motors[joint].connection.RotorAngle();
            const auto speed = motors[joint].connection.RotorSpeed();
            if (!angle.has_value() || !speed.has_value())
                return;
            states[joint] = { *angle, *speed };
        }

        const auto torques = coupling->ShaftTorques(states, collision);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            motors[joint].connection.ExternalTorque(torques[joint]);
    }

    void VirtualArm::Send()
    {
        for (auto& motor : motors)
            if (motor.socket.has_value())
            {
                const auto outgoing = motor.connection.TakeOutgoing();
                motor.socket->Write(outgoing);
            }
    }

    void VirtualArm::Publish()
    {
        std::scoped_lock lock{ mutex };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            auto& motor = motors[joint];
            auto& report = reports[joint];
            const bool joined = motor.socket.has_value() && motor.connection.Introduction().has_value() && motor.connection.LastStatus().has_value();
            report.phase = PhaseOf(motor.process, joined);
            report.exitCode = report.phase == MotorPhase::ended ? motor.process.Ended() : std::nullopt;
            if (report.plant != motor.start.plant)
                report.plant = motor.start.plant;
        }

        if (!up && AllIntroduced())
        {
            up = true;
            changed.notify_all();
        }
    }

    void VirtualArm::Report(std::string reason)
    {
        std::scoped_lock lock{ mutex };
        if (!problem.has_value())
            problem = std::move(reason);
        changed.notify_all();
    }

    void VirtualArm::Post(std::function<void()> request)
    {
        std::scoped_lock lock{ mutex };
        requests.push_back(std::move(request));
    }

    void VirtualArm::RunRequests()
    {
        std::vector<std::function<void()>> pending;
        {
            std::scoped_lock lock{ mutex };
            pending.swap(requests);
        }
        for (const auto& request : pending)
            request();
    }

    bool VirtualArm::AllIntroduced() const
    {
        return std::ranges::all_of(motors, [](const Motor& motor)
            {
                return motor.socket.has_value() && motor.connection.Introduction().has_value() && motor.connection.LastStatus().has_value();
            });
    }
}
