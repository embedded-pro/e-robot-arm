#ifndef TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include "tools/simulator/view_models/ToolReadout.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <deque>
#include <optional>

namespace simulator
{
    class ArmSceneModel
    {
    public:
        static constexpr float ghostThresholdDegrees{ 0.5f };
        static constexpr std::size_t trailCapacity{ 500 };
        static constexpr float trailSpacingCentimetres{ 0.2f };
        using Trail = std::deque<std::array<float, 3>>;

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
        const std::string& SingularityWarning() const;
        const ScenePose& ToolNode() const;
        const Trail& ToolTrail() const;
        uint32_t TrailRevision() const;

    private:
        void UpdateNodes(const arm_host::ArmSnapshot& snapshot);
        void UpdateTrail(bool live, bool rebuilt);

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
        std::string singularityWarning;
        ScenePose toolNode{};
        Trail trail;
        uint32_t trailRevision{ 0 };
    };
}

#endif
