#include "tools/simulator/view_models/TeachPresenter.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        TeachRow RowOf(const arm::TeachPoint& point)
        {
            TeachRow row{ std::string{ point.name.begin(), point.name.end() }, {} };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                row.positions[joint] = std::format("{:.3f}", point.joints[joint].Value());
            return row;
        }
    }

    TeachPresenter::TeachPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : post(post)
    {
        Update();
    }

    void TeachPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot)
    {
        snapshot = newSnapshot;
        rows.clear();
        for (std::size_t point = 0; point != snapshot.config.teachPointCount; ++point)
            rows.push_back(RowOf(snapshot.config.teachPoints[point]));
        if (chosen.has_value() && *chosen >= rows.size())
            chosen.reset();
        Update();
    }

    void TeachPresenter::Choose(std::optional<std::size_t> row)
    {
        chosen = row.has_value() && *row < rows.size() ? row : std::nullopt;
        if (chosen.has_value())
            name = rows[*chosen].name;
        Update();
    }

    void TeachPresenter::SetName(std::string_view newName)
    {
        name = std::string{ newName };
        Update();
    }

    void TeachPresenter::Teach() const
    {
        if (teachState.enabled)
            Post(arm_host::Teach{ name });
    }

    void TeachPresenter::GoTo() const
    {
        if (goToState.enabled)
            Post(arm_host::GoTo{ rows[*chosen].name });
    }

    const std::vector<TeachRow>& TeachPresenter::Rows() const
    {
        return rows;
    }

    std::optional<std::size_t> TeachPresenter::Chosen() const
    {
        return chosen;
    }

    const std::string& TeachPresenter::Name() const
    {
        return name;
    }

    const ControlState& TeachPresenter::TeachState() const
    {
        return teachState;
    }

    const ControlState& TeachPresenter::GoToState() const
    {
        return goToState;
    }

    const std::string& TeachPresenter::Hint() const
    {
        return hint;
    }

    void TeachPresenter::Update()
    {
        const auto replaces = Taught(name);
        if (!snapshot.targetReady)
            teachState = { false, "Not connected" };
        else if (snapshot.moving)
            teachState = { false, "Moving: wait or abort" };
        else if (name.empty() || name.size() > arm::teachPointNameSize)
            teachState = { false, "Name it (1–16 characters)" };
        else if (!replaces && rows.size() >= arm::maxTeachPoints)
            teachState = { false, "16 points already: replace one" };
        else
            teachState = { true, {} };

        if (!chosen.has_value())
            goToState = { false, "Choose a point" };
        else
            goToState = Evaluate(Control::move, snapshot);

        hint = teachState.enabled && replaces ? std::format("Teach replaces \"{}\"", name) : std::string{};
    }

    bool TeachPresenter::Taught(const std::string& candidate) const
    {
        return std::ranges::any_of(rows, [&candidate](const TeachRow& row)
            {
                return row.name == candidate;
            });
    }

    void TeachPresenter::Post(const arm_host::ArmCommand& command) const
    {
        if (post)
            post(command);
    }
}
