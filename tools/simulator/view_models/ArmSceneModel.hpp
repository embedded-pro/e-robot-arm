#ifndef TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_MODEL_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include "tools/simulator/view_models/DesktopArm.hpp"
#include "tools/simulator/view_models/ToolReadout.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace simulator
{
    class ArmSceneModel
    {
    public:
        static constexpr float ghostThresholdDegrees{ 0.5f };
        static constexpr std::size_t trailCapacity{ 500 };
        static constexpr float trailSpacingCentimetres{ 0.2f };
        static constexpr std::size_t limitArcPoints{ 33 };
        static constexpr float limitArcShare{ 0.08f };
        using Trail = std::deque<std::array<float, 3>>;
        using BoxList = std::vector<SceneBox>;

        void Update(const arm_host::ArmSnapshot& snapshot);
        void SetTarget(const std::optional<arm::JointPositions>& joints);
        bool Select(int pickedJoint);
        void ClearSelection();

        const std::array<float, arm::jointCount>& MeasuredDegrees() const;
        const std::array<float, arm::jointCount>& CommandedDegrees() const;
        bool GhostVisible() const;
        std::optional<std::size_t> Selected() const;
        ui::theme::StatusLevel Level(std::size_t joint) const;
        const SceneNodes& MeasuredNodes() const;
        const SceneNodes& CommandedNodes() const;
        const ArmMeshes& Meshes() const;
        uint32_t MeshesRevision() const;
        float Extent() const;
        const std::string& SingularityWarning() const;
        const ScenePose& ToolNode() const;
        const Trail& ToolTrail() const;
        uint32_t TrailRevision() const;
        const BoxList& Boxes() const;
        const std::optional<ScenePose>& TargetNode() const;
        const std::vector<std::array<float, 3>>& LimitArc() const;
        uint32_t LimitArcRevision() const;
        uint32_t BoxesRevision() const;

    private:
        void UpdateNodes(const arm_host::ArmSnapshot& snapshot);
        void UpdateTrail(bool live, bool rebuilt);
        void UpdateBoxes(const arm_host::ArmSnapshot& snapshot);
        void UpdateTarget(const arm::ArmModel& model, bool live);
        void UpdateLimitArc(const arm::ArmModel& model, const arm_host::ArmSnapshot& snapshot, const arm::JointPositions& measuredJoints);

        std::array<float, arm::jointCount> measured{};
        std::array<float, arm::jointCount> commanded{};
        std::array<ui::theme::StatusLevel, arm::jointCount> levels{};
        bool ghostVisible{ false };
        std::optional<std::size_t> selected;
        SceneNodes measuredNodes{};
        SceneNodes commandedNodes{};
        ArmMeshes meshes{};
        std::optional<arm::ArmGeometry> builtFor;
        uint32_t meshesRevision{ 0 };
        float extent{ 0.0f };
        std::string singularityWarning;
        ScenePose toolNode{};
        Trail trail;
        uint32_t trailRevision{ 0 };
        BoxList boxes;
        std::optional<arm::JointPositions> target;
        std::optional<ScenePose> targetNode;
        std::vector<std::array<float, 3>> limitArc;
        uint32_t limitArcRevision{ 0 };
        uint32_t boxesRevision{ 0 };
    };
}

#endif
