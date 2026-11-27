#include "tools/simulator/view_models/ScenePicking.hpp"

namespace simulator
{
    namespace
    {
        constexpr float centimetres{ 100.0f };

        float Dot(const ScenePoint& a, const ScenePoint& b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }
    }

    math::Vector3<float> ToBase(const ScenePoint& point)
    {
        return math::Vector3<float>{ point[0] / centimetres, -point[2] / centimetres, point[1] / centimetres };
    }

    ScenePoint ToScenePoint(const math::Vector3<float>& point)
    {
        return { point[0] * centimetres, point[2] * centimetres, -point[1] * centimetres };
    }

    ScenePoint Nearest(const SceneRay& ray, const ScenePoint& point)
    {
        const auto length = Dot(ray.direction, ray.direction);
        const ScenePoint offset{ point[0] - ray.origin[0], point[1] - ray.origin[1], point[2] - ray.origin[2] };
        const auto along = length > 0.0f ? Dot(offset, ray.direction) / length : 0.0f;
        return { ray.origin[0] + along * ray.direction[0], ray.origin[1] + along * ray.direction[1], ray.origin[2] + along * ray.direction[2] };
    }
}
