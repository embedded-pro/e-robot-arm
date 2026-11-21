#pragma once

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "ui/core/Callback.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/FormView.hpp"
#include <array>
#include <string>

namespace simulator
{
    struct DisturbanceField
    {
        static constexpr ui::model::FieldId joint{ 0 };
        static constexpr ui::model::FieldId torque{ 1 };
    };

    struct DisturbanceAction
    {
        static constexpr ui::model::ActionId injectFault{ 0 };
        static constexpr ui::model::ActionId silence{ 1 };
        static constexpr ui::model::ActionId resume{ 2 };
        static constexpr ui::model::ActionId applyTorque{ 3 };
        static constexpr ui::model::ActionId clearTorque{ 4 };
        static constexpr ui::model::ActionId powerOff{ 5 };
    };

    inline constexpr std::size_t disturbanceActionCount{ 6 };

    class DisturbancePresenter
    {
    public:
        DisturbancePresenter(ui::shell::FormView& view, const ui::Callback<void(const arm_host::ArmCommand& command)>& post);
        DisturbancePresenter(const DisturbancePresenter& other) = delete;
        DisturbancePresenter& operator=(const DisturbancePresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot);
        void SelectJoint(std::size_t joint);

        bool Visible() const;
        std::size_t Joint() const;
        bool Enabled(ui::model::ActionId action) const;
        const std::string& Active() const;
        const std::string& Hint() const;

    private:
        struct Form
        {
            Form();
            Form(const Form& other) = delete;
            Form& operator=(const Form& other) = delete;

            ui::model::FormSpec spec;
            std::array<ui::model::FieldValue, 2> values{};
            ui::model::FormModel model;
        };

        void OnAction(ui::model::ActionId action);
        std::string Refusal(ui::model::ActionId action) const;
        std::string OfferRefusal(ui::model::ActionId action) const;
        void Update();

        ui::shell::FormView& view;
        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        Form form;
        arm_host::ArmSnapshot snapshot;
        std::array<std::string, disturbanceActionCount> refusals{};
        std::string active;
        std::string hint;
    };
}
