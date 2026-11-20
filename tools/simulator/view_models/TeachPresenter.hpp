#ifndef TOOLS_SIMULATOR_VIEW_MODELS_TEACH_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_TEACH_PRESENTER_HPP

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "ui/core/Callback.hpp"
#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    struct TeachRow
    {
        std::string name;
        std::array<std::string, arm::jointCount> positions;

        friend bool operator==(const TeachRow& lhs, const TeachRow& rhs) = default;
    };

    class TeachPresenter
    {
    public:
        explicit TeachPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);
        TeachPresenter(const TeachPresenter& other) = delete;
        TeachPresenter& operator=(const TeachPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot);
        void Choose(std::optional<std::size_t> row);
        void SetName(std::string_view name);
        void Teach() const;
        void GoTo() const;

        const std::vector<TeachRow>& Rows() const;
        std::optional<std::size_t> Chosen() const;
        const std::string& Name() const;
        const ControlState& TeachState() const;
        const ControlState& GoToState() const;
        const std::string& Hint() const;

    private:
        void Update();
        bool Taught(const std::string& name) const;
        void Post(const arm_host::ArmCommand& command) const;

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        arm_host::ArmSnapshot snapshot;
        std::vector<TeachRow> rows;
        std::optional<std::size_t> chosen;
        std::string name;
        ControlState teachState{ false, "Not connected" };
        ControlState goToState{ false, "Choose a point" };
        std::string hint;
    };
}

#endif
