#include "tools/virtual_arm/plant_port/MotorConnection.hpp"
#include <cmath>
#include <format>
#include <numbers>
#include <utility>

namespace virtual_arm::plant_port
{
    void MotorConnection::Receive(std::span<const uint8_t> bytes)
    {
        if (failure.has_value())
            return;

        assembler.Append(bytes);
        while (!failure.has_value())
        {
            const auto frame = assembler.Next();
            if (!frame.has_value())
                return;
            Handle(*frame);
        }
    }

    const std::optional<Hello>& MotorConnection::Introduction() const
    {
        return introduction;
    }

    const std::optional<std::string>& MotorConnection::Failure() const
    {
        return failure;
    }

    std::optional<float> MotorConnection::RotorAngle() const
    {
        return rawAngle.has_value() ? std::optional<float>{ unwrappedAngle } : std::nullopt;
    }

    std::optional<float> MotorConnection::RotorSpeed() const
    {
        return speed;
    }

    const std::optional<Status>& MotorConnection::LastStatus() const
    {
        return status;
    }

    const std::optional<Refused>& MotorConnection::LastRefusal() const
    {
        return refusal;
    }

    const std::deque<std::string>& MotorConnection::RecentLog() const
    {
        return log;
    }

    void MotorConnection::Stream(bool on)
    {
        Queue(EncodeStream(on));
    }

    void MotorConnection::Pause()
    {
        Queue(EncodeFrame(MessageType::pause));
    }

    void MotorConnection::Resume()
    {
        Queue(EncodeFrame(MessageType::resume));
    }

    void MotorConnection::ExternalTorque(float newtonMeter)
    {
        Queue(EncodeConstantExternalTorque(newtonMeter));
    }

    void MotorConnection::ClearAll()
    {
        Queue(EncodeFrame(MessageType::clearAll));
    }

    std::vector<uint8_t> MotorConnection::TakeOutgoing()
    {
        return std::exchange(outgoing, {});
    }

    void MotorConnection::Handle(const Frame& frame)
    {
        if (!introduction.has_value() && frame.type != static_cast<uint8_t>(MessageType::hello))
            return Fail(std::format("frame 0x{:02x} before hello", frame.type));

        switch (static_cast<MessageType>(frame.type))
        {
            case MessageType::hello:
                if (const auto hello = DecodeHello(frame))
                    return Introduce(*hello);
                break;
            case MessageType::status:
                if ((status = DecodeStatus(frame)))
                    return;
                break;
            case MessageType::refused:
                if ((refusal = DecodeRefused(frame)))
                    return;
                break;
            case MessageType::logLine:
                if (auto line = DecodeLogLine(frame))
                {
                    log.push_back(std::move(*line));
                    if (log.size() > keptLogLines)
                        log.pop_front();
                    return;
                }
                break;
            case MessageType::window:
                if (const auto window = DecodeWindow(frame))
                    return Follow(*window);
                break;
            default:
                return;
        }

        Fail(std::format("malformed frame 0x{:02x}", frame.type));
    }

    void MotorConnection::Introduce(const Hello& hello)
    {
        if (hello.version != protocolVersion)
            return Fail(std::format("plant port protocol version {}, expected {}", hello.version, protocolVersion));
        introduction = hello;
    }

    void MotorConnection::Follow(const Window& window)
    {
        const auto angle = window.last[rotorAngleSignal];
        if (rawAngle.has_value())
            unwrappedAngle += std::remainder(angle - *rawAngle, 2.0f * std::numbers::pi_v<float>);
        else
            unwrappedAngle = angle;
        rawAngle = angle;
        speed = window.last[rotorSpeedSignal];
    }

    void MotorConnection::Fail(std::string reason)
    {
        failure = std::move(reason);
    }

    void MotorConnection::Queue(const std::vector<uint8_t>& frame)
    {
        outgoing.insert(outgoing.end(), frame.begin(), frame.end());
    }
}
