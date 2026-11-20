#ifndef TOOLS_SIMULATOR_VIEW_MODELS_JOG_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_JOG_PRESENTER_HPP

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "ui/core/Callback.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace simulator
{
    enum class JogSpace : uint8_t
    {
        joints,
        base,
        tool
    };

    inline constexpr std::size_t jogAxisCount{ 6 };
    inline constexpr int defaultJogShare{ 10 };

    struct JogRow
    {
        std::string label;
        std::string speed;

        friend bool operator==(const JogRow& lhs, const JogRow& rhs) = default;
    };

    class JogPresenter
    {
    public:
        using Clock = std::chrono::steady_clock;

        explicit JogPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);
        JogPresenter(const JogPresenter& other) = delete;
        JogPresenter& operator=(const JogPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, Clock::time_point now);
        void SetSpace(JogSpace space);
        void SetShare(int percent);
        void ChooseAxis(std::size_t axis);
        void Press(std::size_t axis, int direction, Clock::time_point now);
        void Release();

        JogSpace Space() const;
        int Share() const;
        std::optional<std::size_t> ChosenAxis() const;
        bool Held() const;
        const ControlState& JogState() const;
        const ControlState& SpaceState() const;
        const std::array<JogRow, jogAxisCount>& Rows() const;
        const std::string& Status() const;

    private:
        struct Hold
        {
            std::size_t axis;
            int direction;
            Clock::time_point sent;
        };

        void Update();
        ControlState Evaluate() const;
        float Speed(std::size_t axis) const;
        Clock::duration RefreshPeriod() const;
        void Send(std::size_t axis, float speed) const;

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        arm_host::ArmSnapshot snapshot;
        JogSpace space{ JogSpace::joints };
        int share{ defaultJogShare };
        std::optional<std::size_t> chosen;
        std::optional<Hold> hold;
        ControlState jogState{ false, "Not connected" };
        ControlState spaceState{ true, {} };
        std::array<JogRow, jogAxisCount> rows{};
        std::string status;
    };
}

#endif
