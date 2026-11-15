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

        bool HubPortUsable()
        {
            for (const auto socket : { services::loopback::Listen(services::loopback::virtualCanHubPort), services::loopback::Connect(services::loopback::virtualCanHubPort) })
                if (socket != services::loopback::invalidSocket)
                {
                    services::loopback::Close(socket);
                    return true;
                }
            return false;
        }

        bool WaitForHubPort(std::chrono::milliseconds timeout)
        {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (!HubPortUsable())
            {
                if (std::chrono::steady_clock::now() >= deadline)
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds{ 250 });
            }
            return true;
        }
    }

    VirtualArm::VirtualArm(const arm::ArmConfig& config, VirtualArmOptions options)
        : config(config)
        , options(std::move(options))
        , coupling(config)
    {}

    VirtualArm::~VirtualArm()
    {
        Stop();
    }

    std::optional<std::string> VirtualArm::Start()
    {
        const auto plan = PlanVirtualArm(config);
        if (const auto* refusal = std::get_if<std::string>(&plan))
            return *refusal;

        std::error_code error;
        std::filesystem::create_directories(options.workDirectory, error);
        if (error)
            return std::format("cannot create {}: {}", options.workDirectory.string(), error.message());

        if (!WaitForHubPort(options.hubPortTimeout))
            return std::format("the virtual CAN hub port {} is still held by a closed connection", services::loopback::virtualCanHubPort);

        listener.emplace();
        if (!listener->Port().has_value())
            return std::string{ "cannot listen on a loopback port for the plant ports" };

        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            motors[joint].start = std::get<VirtualArmPlan>(plan)[joint];
            const auto log = std::filesystem::absolute(options.workDirectory / std::format("joint{}.log", joint + 1));
            if (!motors[joint].process.Start(options.engine, EngineArguments(motors[joint].start, options.bus, *listener->Port()), options.workDirectory, log))
            {
                Stop();
                return std::format("cannot start {}", options.engine.string());
            }
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

        if (AllIntroduced())
        {
            std::scoped_lock lock{ mutex };
            if (!up)
            {
                up = true;
                changed.notify_all();
            }
        }
    }

    void VirtualArm::Watch()
    {
        for (auto& motor : motors)
            if (const auto exitCode = motor.process.Ended(); exitCode.has_value() && motor.socket.has_value())
            {
                Report(std::format("joint {}'s motor ended unasked, exit code {}{}", motor.start.joint + 1, *exitCode, LastLines(motor.connection)));
                motor.socket.reset();
            }
            else if (exitCode.has_value() && !motor.connection.Introduction().has_value())
                Report(std::format("joint {}'s motor ended before it came up, exit code {}", motor.start.joint + 1, *exitCode));
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

        const auto torques = coupling.ShaftTorques(states, collision);
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
