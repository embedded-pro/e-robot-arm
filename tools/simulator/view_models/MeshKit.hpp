#ifndef TOOLS_SIMULATOR_VIEW_MODELS_MESH_KIT_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_MESH_KIT_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace simulator::mesh
{
    struct Vec3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };

        bool operator==(const Vec3& other) const = default;
    };

    struct Vertex
    {
        Vec3 position;
        Vec3 normal;

        bool operator==(const Vertex& other) const = default;
    };

    struct Frame
    {
        Vec3 origin{};
        Vec3 xAxis{ 1.0f, 0.0f, 0.0f };
        Vec3 yAxis{ 0.0f, 1.0f, 0.0f };
        Vec3 zAxis{ 0.0f, 0.0f, 1.0f };

        Vec3 Apply(const Vec3& point) const;
        Vec3 Rotate(const Vec3& direction) const;
    };

    Frame At(const Vec3& origin);
    Frame Along(const Vec3& origin, const Vec3& axis);

    struct Bounds
    {
        Vec3 lowest;
        Vec3 highest;
    };

    class Mesh
    {
    public:
        void Add(const Mesh& part, const Frame& frame = {});
        void AddTriangle(const Vertex& a, const Vertex& b, const Vertex& c);
        void AddQuad(const Vertex& a, const Vertex& b, const Vertex& c, const Vertex& d);

        const std::vector<Vertex>& Vertices() const;
        const std::vector<uint32_t>& Indices() const;
        std::size_t Triangles() const;
        bool Empty() const;
        Bounds Extent() const;

        bool operator==(const Mesh& other) const = default;

    private:
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
    };

    using Point2 = std::array<float, 2>;
    using Outline = std::vector<Point2>;

    Mesh Lathe(const Outline& profile, std::size_t segments);
    Mesh Extrude(const Outline& outline, float bottom, float top, float smoothAngleDegrees = 30.0f);

    Outline RoundedRectangle(float width, float height, float radius, std::size_t cornerSegments);
    Outline Stadium(float length, float width, std::size_t endSegments);
    Outline RegularPolygon(std::size_t sides, float radius);
}

#endif
