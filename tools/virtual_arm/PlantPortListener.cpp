#include "tools/virtual_arm/PlantPortListener.hpp"
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace virtual_arm
{
    namespace
    {
        template<class Address>
        sockaddr* AsSocketAddress(Address& address)
        {
            return static_cast<sockaddr*>(static_cast<void*>(&address));
        }

        int NonBlocking(int descriptor)
        {
            if (descriptor >= 0)
                fcntl(descriptor, F_SETFL, fcntl(descriptor, F_GETFL, 0) | O_NONBLOCK);
            return descriptor;
        }

        int LoopbackListener()
        {
            const auto descriptor = NonBlocking(socket(AF_INET, SOCK_STREAM, 0));
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            address.sin_port = 0;
            if (descriptor < 0 || bind(descriptor, AsSocketAddress(address), sizeof(address)) != 0 || listen(descriptor, 8) != 0)
            {
                if (descriptor >= 0)
                    close(descriptor);
                return -1;
            }
            return descriptor;
        }
    }

    Socket::Socket(int descriptor)
        : descriptor(descriptor)
    {}

    Socket::Socket(Socket&& other) noexcept
        : descriptor(std::exchange(other.descriptor, -1))
        , pending(std::move(other.pending))
    {}

    Socket& Socket::operator=(Socket&& other) noexcept
    {
        Close();
        descriptor = std::exchange(other.descriptor, -1);
        pending = std::move(other.pending);
        return *this;
    }

    Socket::~Socket()
    {
        Close();
    }

    int Socket::Descriptor() const
    {
        return descriptor;
    }

    bool Socket::Open() const
    {
        return descriptor >= 0;
    }

    std::vector<uint8_t> Socket::Read()
    {
        std::vector<uint8_t> received;
        std::array<uint8_t, 4096> buffer{};
        while (Open())
        {
            const auto count = recv(descriptor, buffer.data(), buffer.size(), 0);
            if (count > 0)
                received.insert(received.end(), buffer.begin(), buffer.begin() + count);
            else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                Close();
            else
                break;
        }
        return received;
    }

    void Socket::Write(std::span<const uint8_t> bytes)
    {
        pending.insert(pending.end(), bytes.begin(), bytes.end());
        Flush();
    }

    void Socket::Flush()
    {
        while (Open() && !pending.empty())
        {
            const auto count = send(descriptor, pending.data(), pending.size(), MSG_NOSIGNAL);
            if (count > 0)
                pending.erase(pending.begin(), pending.begin() + count);
            else if (errno == EAGAIN || errno == EWOULDBLOCK)
                return;
            else
                Close();
        }
    }

    void Socket::Close()
    {
        if (descriptor >= 0)
            close(descriptor);
        descriptor = -1;
    }

    PlantPortListener::PlantPortListener()
        : listener(LoopbackListener())
    {
        sockaddr_in address{};
        socklen_t length{ sizeof(address) };
        if (listener.Open() && getsockname(listener.Descriptor(), AsSocketAddress(address), &length) == 0)
            port = ntohs(address.sin_port);
    }

    std::optional<uint16_t> PlantPortListener::Port() const
    {
        return port;
    }

    int PlantPortListener::Descriptor() const
    {
        return listener.Descriptor();
    }

    std::optional<Socket> PlantPortListener::Accept()
    {
        const auto descriptor = NonBlocking(accept(listener.Descriptor(), nullptr, nullptr));
        if (descriptor < 0)
            return std::nullopt;

        const int noDelay{ 1 };
        setsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
        return Socket{ descriptor };
    }
}
