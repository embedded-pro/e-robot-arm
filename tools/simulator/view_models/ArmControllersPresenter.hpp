#pragma once

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "ui/core/Callback.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    enum class ArmControllerAction : uint8_t
    {
        select,
        enable,
        disable
    };

    inline constexpr std::size_t armControllerActionCount{ 3 };

    struct ArmControllerRow
    {
        std::string name;
        std::string state;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
        std::string parameters;
        bool enabled{ false };

        friend bool operator==(const ArmControllerRow& lhs, const ArmControllerRow& rhs) = default;
    };

    class ArmControllersPresenter
    {
    public:
        explicit ArmControllersPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);
        ArmControllersPresenter(const ArmControllersPresenter& other) = delete;
        ArmControllersPresenter& operator=(const ArmControllersPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot);
        void Choose(std::optional<std::size_t> row);
        void Trigger(ArmControllerAction action);
        void SetDisablePrompt(const ui::Callback<void(std::string_view prompt)>& prompt);
        void ConfirmDisable();

        const std::vector<ArmControllerRow>& Rows() const;
        std::optional<std::size_t> Chosen() const;
        const ControlState& State(ArmControllerAction action) const;
        const std::string& Hint() const;

    private:
        void Update();
        ControlState Evaluate(ArmControllerAction action) const;
        void Post(const arm_host::ArmCommand& command) const;

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        ui::Callback<void(std::string_view prompt)> disablePrompt;
        arm_host::ArmSnapshot snapshot;
        std::vector<ArmControllerRow> rows;
        std::optional<std::size_t> chosen;
        std::array<ControlState, armControllerActionCount> states{};
        std::string hint;
    };
}
