#ifndef TOOLS_VIRTUAL_ARM_VIRTUAL_ARM_HPP
#define TOOLS_VIRTUAL_ARM_VIRTUAL_ARM_HPP

#include "tools/virtual_arm/ArmCoupling.hpp"
#include "tools/virtual_arm/MotorProcess.hpp"
#include "tools/virtual_arm/PlantPortListener.hpp"
#include "tools/virtual_arm/VirtualArmPlan.hpp"
#include "tools/virtual_arm/plant_port/MotorConnection.hpp"
#include <array>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace virtual_arm
{
    struct VirtualArmOptions
    {
        std::filesystem::path engine{ "e-foc-virtual-motor" };
        std::string bus{ defaultBus };
        std::filesystem::path workDirectory{ "virtual-arm" };
        std::chrono::milliseconds upTimeout{ 10000 };
        std::chrono::milliseconds hubPortTimeout{ 70000 };
        uint16_t hubPort{ 29536 };
    };

    class VirtualArm
    {
    public:
        VirtualArm(const arm::ArmConfig& config, VirtualArmOptions options);
        VirtualArm(const VirtualArm& other) = delete;
        VirtualArm& operator=(const VirtualArm& other) = delete;
        ~VirtualArm();

        std::optional<std::string> Start();
        void Stop();
        bool Up() const;
        const std::string& Bus() const;
        std::optional<std::string> Problem() const;

        void Silence(std::size_t joint, bool silent);
        void Collide(std::size_t joint, foc::NewtonMeter torque);

    private:
        struct Motor
        {
            MotorStart start;
            MotorProcess process;
            std::optional<Socket> socket;
            plant_port::MotorConnection connection;
        };

        struct Arrival
        {
            Socket socket;
            plant_port::MotorConnection connection;
        };

        std::optional<std::string> StartMotors(const VirtualArmPlan& plan);
        std::optional<std::string> StartMotor(const MotorStart& start);
        std::optional<std::string> WaitForHub(Motor& host);
        std::optional<std::string> WaitUntilUp();
        void Run(const std::stop_token& stop);
        void Poll();
        void Receive();
        void Identify();
        void Watch();
        void Couple();
        void Send();
        void Report(std::string problem);
        void Post(std::function<void()> request);
        void RunRequests();
        bool AllIntroduced() const;

        arm::ArmConfig config;
        VirtualArmOptions options;
        ArmCoupling coupling;
        std::optional<PlantPortListener> listener;
        std::array<Motor, arm::jointCount> motors{};
        std::vector<Arrival> arrivals;
        arm::JointTorques collision{};
        std::chrono::steady_clock::time_point nextCoupling{};

        mutable std::mutex mutex;
        std::condition_variable changed;
        bool up{ false };
        std::optional<std::string> problem;
        std::vector<std::function<void()>> requests;
        std::jthread worker;
    };
}

#endif
