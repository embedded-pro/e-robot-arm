#ifndef CORE_ARM_MODEL_ARM_GEOMETRY_HPP
#define CORE_ARM_MODEL_ARM_GEOMETRY_HPP

#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/AnalyticalIkOpw.hpp"

namespace arm
{
    struct Placement
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
        float roll{ 0.0f };
        float pitch{ 0.0f };
        float yaw{ 0.0f };
    };

    struct ArmGeometry
    {
        kinematics::OpwParameters<float> opw{};
        Placement tool{};
        Placement base{};
    };

    inline ArmGeometry ReferenceGeometry()
    {
        ArmGeometry geometry{};
        geometry.opw.a1 = 0.02f;
        geometry.opw.c1 = 0.15f;
        geometry.opw.c2 = 0.25f;
        geometry.opw.c3 = 0.25f;
        geometry.opw.c4 = 0.06f;
        return geometry;
    }

    math::SE3Transform<float> ToTransform(const Placement& placement);
    Placement ToPlacement(const math::SE3Transform<float>& transform);
}

#endif
