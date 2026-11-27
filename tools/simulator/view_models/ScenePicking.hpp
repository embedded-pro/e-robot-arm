#pragma once

#include "numerical/math/Geometry3D.hpp"
#include <array>

namespace simulator
{
    using ScenePoint = std::array<float, 3>;

    struct SceneRay
    {
        ScenePoint origin{};
        ScenePoint direction{};
    };

    // The scene is in centimetres, y up; the arm base frame is in metres, z up.
    math::Vector3<float> ToBase(const ScenePoint& point);
    ScenePoint ToScenePoint(const math::Vector3<float>& point);
    ScenePoint Nearest(const SceneRay& ray, const ScenePoint& point);

    struct BaseRay
    {
        math::Vector3<float> origin{};
        math::Vector3<float> direction{};
    };

    // The ray in the arm base frame, with a unit direction.
    BaseRay ToBaseRay(const SceneRay& ray);
}
