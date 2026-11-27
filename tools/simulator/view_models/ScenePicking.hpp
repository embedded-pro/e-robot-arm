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
}
