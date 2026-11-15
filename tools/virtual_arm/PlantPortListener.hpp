#ifndef TOOLS_VIRTUAL_ARM_PLANT_PORT_LISTENER_HPP
#define TOOLS_VIRTUAL_ARM_PLANT_PORT_LISTENER_HPP

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace virtual_arm
{
    class Socket
    {
    public:
        explicit Socket(int descriptor);
        Socket(Socket&& other) noexcept;
        Socket& operator=(Socket&& other) noexcept;
        Socket(const Socket& other) = delete;
        Socket& operator=(const Socket& other) = delete;
        ~Socket();

        int Descriptor() const;
        bool Open() const;
        std::vector<uint8_t> Read();
        void Write(std::span<const uint8_t> bytes);
        void Flush();
        void Close();

    private:
        int descriptor{ -1 };
        std::vector<uint8_t> pending;
    };

    class PlantPortListener
    {
    public:
        PlantPortListener();

        std::optional<uint16_t> Port() const;
        int Descriptor() const;
        std::optional<Socket> Accept();

    private:
        Socket listener;
        std::optional<uint16_t> port;
    };
}

#endif
