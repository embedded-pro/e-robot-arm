#ifndef TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <optional>

namespace simulator
{
    class ArmSceneModel
    {
    public:
        static constexpr float ghostThresholdDegrees{ 0.5f };

        void Update(const arm_host::ArmSnapshot& snapshot);
        bool Select(int pickedJoint);
        void ClearSelection();

        const std::array<float, arm::jointCount>& MeasuredDegrees() const;
        const std::array<float, arm::jointCount>& CommandedDegrees() const;
        bool GhostVisible() const;
        std::optional<std::size_t> Selected() const;
        ui::theme::StatusLevel Level(std::size_t joint) const;
        const SceneNodes& MeasuredNodes() const;
        const SceneNodes& CommandedNodes() const;
        const SceneParts& Parts() const;
        uint32_t PartsRevision() const;
        float Extent() const;

    private:
        void UpdateNodes(const arm_host::ArmSnapshot& snapshot);

        std::array<float, arm::jointCount> measured{};
        std::array<float, arm::jointCount> commanded{};
        std::array<ui::theme::StatusLevel, arm::jointCount> levels{};
        bool ghostVisible{ false };
        std::optional<std::size_t> selected;
        SceneNodes measuredNodes{};
        SceneNodes commandedNodes{};
        SceneParts parts{};
        uint32_t partsRevision{ 0 };
        float extent{ 0.0f };
    };
}

#endif
