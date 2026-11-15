#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include <arpa/inet.h>
#include <array>
#include <bit>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
    using namespace virtual_arm::plant_port;

    struct Options
    {
        uint8_t node{ 0 };
        uint16_t plantPort{ 0 };
        std::string memory{ "motor.mem" };
        std::string plant{ "teknic" };
        uint16_t hubPort{ 0 };
        bool hostBus{ false };
    };

    Options Parse(int count, char** arguments)
    {
        Options options;
        for (int index = 1; index < count; ++index)
            options.hostBus = options.hostBus || std::string{ arguments[index] } == "--host-bus";
        for (int index = 1; index + 1 < count; ++index)
        {
            const std::string name{ arguments[index] };
            const std::string value{ arguments[index + 1] };
            if (name == "--node")
                options.node = static_cast<uint8_t>(std::stoi(value));
            else if (name == "--plant-port")
                options.plantPort = static_cast<uint16_t>(std::stoi(value));
            else if (name == "--memory")
                options.memory = value;
            else if (name == "--plant")
                options.plant = value;
            else if (name == "--hub-port")
                options.hubPort = static_cast<uint16_t>(std::stoi(value));
        }
        return options;
    }

    void Put(std::vector<uint8_t>& bytes, uint64_t value, int size)
    {
        for (int index = 0; index != size; ++index)
            bytes.push_back(static_cast<uint8_t>(value >> (8 * index)));
    }

    std::vector<uint8_t> HelloFrame(const Options& options, uint16_t version)
    {
        std::vector<uint8_t> payload;
        Put(payload, version, 2);
        payload.push_back(options.node);
        payload.push_back(3);
        payload.insert(payload.end(), { 'a', 'r', 'm' });
        payload.push_back(static_cast<uint8_t>(options.plant.size()));
        payload.insert(payload.end(), options.plant.begin(), options.plant.end());
        Put(payload, 20000, 4);
        return EncodeFrame(MessageType::hello, payload);
    }

    std::vector<uint8_t> StatusFrame()
    {
        std::vector<uint8_t> payload;
        Put(payload, 2000, 8);
        Put(payload, std::bit_cast<uint32_t>(1.0f), 4);
        Put(payload, 0, 4);
        payload.insert(payload.end(), { 0, 0b001, 0 });
        return EncodeFrame(MessageType::status, payload);
    }

    std::vector<uint8_t> WindowFrame(uint32_t sequence, float angle)
    {
        std::vector<uint8_t> payload;
        Put(payload, sequence, 4);
        Put(payload, 10ull * sequence, 8);
        payload.push_back(10);
        for (std::size_t signal = 0; signal != 3 * signalCount; ++signal)
            Put(payload, std::bit_cast<uint32_t>(signal == 2 * signalCount + rotorAngleSignal ? angle : 0.0f), 4);
        return EncodeFrame(MessageType::window, payload);
    }

    void Record(const std::string& file, const std::string& text)
    {
        std::ofstream{ file, std::ios::trunc } << text;
    }

    sockaddr_in Loopback(uint16_t port)
    {
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        return address;
    }

    int HostHub(uint16_t port)
    {
        const auto descriptor = socket(AF_INET, SOCK_STREAM, 0);
        int enabled{ 1 };
        setsockopt(descriptor, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
        const auto address = Loopback(port);
        bind(descriptor, static_cast<const sockaddr*>(static_cast<const void*>(&address)), sizeof(address));
        listen(descriptor, 8);
        return descriptor;
    }

    bool HubListening(uint16_t port)
    {
        const auto descriptor = socket(AF_INET, SOCK_STREAM, 0);
        const auto address = Loopback(port);
        const bool listening = connect(descriptor, static_cast<const sockaddr*>(static_cast<const void*>(&address)), sizeof(address)) == 0;
        close(descriptor);
        return listening;
    }

    int Connect(uint16_t port)
    {
        const auto descriptor = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        return connect(descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0 ? descriptor : -1;
    }
}

int main(int count, char** arguments)
{
    const auto options = Parse(count, arguments);
    const auto* version = std::getenv("FAKE_MOTOR_VERSION");
    const auto* exitAfter = std::getenv("FAKE_MOTOR_EXIT_AFTER_MS");
    const auto descriptor = Connect(options.plantPort);
    if (descriptor < 0)
        return 2;

    const auto hello = HelloFrame(options, version != nullptr ? static_cast<uint16_t>(std::atoi(version)) : protocolVersion);
    send(descriptor, hello.data(), hello.size(), MSG_NOSIGNAL);
    if (options.hostBus)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{ 200 });
        HostHub(options.hubPort);
    }
    else if (!HubListening(options.hubPort))
        return 4;
    const auto status = StatusFrame();
    send(descriptor, status.data(), status.size(), MSG_NOSIGNAL);

    const auto start = std::chrono::steady_clock::now();
    const float angle{ 0.1f * static_cast<float>(options.node + 1) };
    FrameAssembler assembler;
    bool streaming{ false };
    uint32_t sequence{ 0 };
    while (true)
    {
        if (exitAfter != nullptr && std::chrono::steady_clock::now() - start > std::chrono::milliseconds{ std::atoi(exitAfter) })
            return 3;

        pollfd readable{ descriptor, POLLIN, 0 };
        if (poll(&readable, 1, 2) > 0)
        {
            std::array<uint8_t, 4096> buffer{};
            const auto received = recv(descriptor, buffer.data(), buffer.size(), 0);
            if (received <= 0)
                return 0;
            assembler.Append(std::span{ buffer }.first(static_cast<std::size_t>(received)));
        }

        while (const auto frame = assembler.Next())
        {
            PayloadReader reader{ frame->payload };
            switch (static_cast<MessageType>(frame->type))
            {
                case MessageType::stream:
                    streaming = reader.U8() != 0;
                    break;
                case MessageType::externalTorque:
                    reader.U8();
                    Record(options.memory + ".torque", std::to_string(reader.F32()));
                    break;
                case MessageType::pause:
                    Record(options.memory + ".paused", "paused");
                    break;
                case MessageType::resume:
                    unlink((options.memory + ".paused").c_str());
                    break;
                default:
                    break;
            }
        }

        if (streaming)
        {
            const auto window = WindowFrame(sequence++, angle);
            send(descriptor, window.data(), window.size(), MSG_NOSIGNAL);
        }
    }
}
