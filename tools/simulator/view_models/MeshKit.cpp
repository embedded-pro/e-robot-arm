#include "tools/simulator/view_models/MeshKit.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace simulator::mesh
{
    namespace
    {
        constexpr float pi{ std::numbers::pi_v<float> };

        Vec3 Add(const Vec3& a, const Vec3& b)
        {
            return { a.x + b.x, a.y + b.y, a.z + b.z };
        }

        Vec3 Scale(const Vec3& a, float factor)
        {
            return { a.x * factor, a.y * factor, a.z * factor };
        }

        Vec3 Cross(const Vec3& a, const Vec3& b)
        {
            return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
        }

        float Dot(const Vec3& a, const Vec3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        Vec3 Normalised(const Vec3& a)
        {
            const auto length = std::sqrt(Dot(a, a));
            return length > 0.0f ? Scale(a, 1.0f / length) : a;
        }

        Point2 EdgeNormal(const Point2& from, const Point2& to)
        {
            const auto dx = to[0] - from[0];
            const auto dy = to[1] - from[1];
            const auto length = std::sqrt(dx * dx + dy * dy);
            return length > 0.0f ? Point2{ dy / length, -dx / length } : Point2{ 0.0f, 0.0f };
        }

        Point2 Blend(const Point2& a, const Point2& b)
        {
            const auto x = a[0] + b[0];
            const auto y = a[1] + b[1];
            const auto length = std::sqrt(x * x + y * y);
            return length > 0.0f ? Point2{ x / length, y / length } : a;
        }

        Outline Arc(float centreX, float centreY, float radius, float from, float to, std::size_t segments)
        {
            Outline arc;
            for (std::size_t step = 0; step <= segments; ++step)
            {
                const auto angle = from + (to - from) * static_cast<float>(step) / static_cast<float>(segments);
                arc.push_back({ centreX + radius * std::cos(angle), centreY + radius * std::sin(angle) });
            }
            return arc;
        }
    }

    Vec3 Frame::Apply(const Vec3& point) const
    {
        return ::simulator::mesh::Add(origin, Rotate(point));
    }

    Vec3 Frame::Rotate(const Vec3& direction) const
    {
        return ::simulator::mesh::Add(::simulator::mesh::Add(Scale(xAxis, direction.x), Scale(yAxis, direction.y)), Scale(zAxis, direction.z));
    }

    Frame At(const Vec3& origin)
    {
        return Frame{ origin };
    }

    Frame Along(const Vec3& origin, const Vec3& axis)
    {
        const auto z = Normalised(axis);
        const Vec3 helper = std::abs(z.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
        const auto x = Normalised(Cross(helper, z));
        return Frame{ origin, x, Cross(z, x), z };
    }

    void Mesh::Add(const Mesh& part, const Frame& frame)
    {
        const auto offset = static_cast<uint32_t>(vertices.size());
        for (const auto& vertex : part.vertices)
            vertices.emplace_back(frame.Apply(vertex.position), Normalised(frame.Rotate(vertex.normal)));
        for (const auto index : part.indices)
            indices.push_back(offset + index);
    }

    void Mesh::AddTriangle(const Vertex& a, const Vertex& b, const Vertex& c)
    {
        const auto first = static_cast<uint32_t>(vertices.size());
        vertices.insert(vertices.end(), { a, b, c });
        indices.insert(indices.end(), { first, first + 1, first + 2 });
    }

    void Mesh::AddQuad(const Vertex& a, const Vertex& b, const Vertex& c, const Vertex& d)
    {
        const auto first = static_cast<uint32_t>(vertices.size());
        vertices.insert(vertices.end(), { a, b, c, d });
        indices.insert(indices.end(), { first, first + 1, first + 2, first, first + 2, first + 3 });
    }

    const std::vector<Vertex>& Mesh::Vertices() const
    {
        return vertices;
    }

    const std::vector<uint32_t>& Mesh::Indices() const
    {
        return indices;
    }

    std::size_t Mesh::Triangles() const
    {
        return indices.size() / 3;
    }

    bool Mesh::Empty() const
    {
        return indices.empty();
    }

    Bounds Mesh::Extent() const
    {
        constexpr auto huge = std::numeric_limits<float>::max();
        Bounds bounds{ { huge, huge, huge }, { -huge, -huge, -huge } };
        for (const auto& vertex : vertices)
        {
            const auto& p = vertex.position;
            bounds.lowest = { std::min(bounds.lowest.x, p.x), std::min(bounds.lowest.y, p.y), std::min(bounds.lowest.z, p.z) };
            bounds.highest = { std::max(bounds.highest.x, p.x), std::max(bounds.highest.y, p.y), std::max(bounds.highest.z, p.z) };
        }
        return bounds;
    }

    Mesh Lathe(const Outline& profile, std::size_t segments)
    {
        Mesh mesh;
        for (std::size_t band = 0; band + 1 < profile.size(); ++band)
        {
            const auto& lower = profile[band];
            const auto& upper = profile[band + 1];
            const auto normal = EdgeNormal(lower, upper);
            for (std::size_t step = 0; step != segments; ++step)
            {
                const auto a0 = 2.0f * pi * static_cast<float>(step) / static_cast<float>(segments);
                const auto a1 = 2.0f * pi * static_cast<float>(step + 1) / static_cast<float>(segments);
                const auto point = [](const Point2& p, float angle)
                {
                    return Vec3{ p[0] * std::cos(angle), p[0] * std::sin(angle), p[1] };
                };
                const auto direction = [&normal](float angle)
                {
                    return Vec3{ normal[0] * std::cos(angle), normal[0] * std::sin(angle), normal[1] };
                };
                mesh.AddQuad({ point(lower, a0), direction(a0) }, { point(lower, a1), direction(a1) }, { point(upper, a1), direction(a1) },
                    { point(upper, a0), direction(a0) });
            }
        }
        return mesh;
    }

    Mesh Extrude(const Outline& outline, float bottom, float top, float smoothAngleDegrees)
    {
        Mesh mesh;
        const auto count = outline.size();
        if (count < 3)
            return mesh;

        const auto smoothCos = std::cos(smoothAngleDegrees * pi / 180.0f);
        std::vector<Point2> edges(count);
        for (std::size_t index = 0; index != count; ++index)
            edges[index] = EdgeNormal(outline[index], outline[(index + 1) % count]);

        const auto cornerNormal = [&edges, count, smoothCos](std::size_t corner, std::size_t edge)
        {
            const auto& before = edges[(corner + count - 1) % count];
            const auto& after = edges[corner % count];
            const bool smooth = before[0] * after[0] + before[1] * after[1] > smoothCos;
            return smooth ? Blend(before, after) : edges[edge];
        };

        for (std::size_t edge = 0; edge != count; ++edge)
        {
            const auto next = (edge + 1) % count;
            const auto n0 = cornerNormal(edge, edge);
            const auto n1 = cornerNormal(next, edge);
            const auto& p0 = outline[edge];
            const auto& p1 = outline[next];
            mesh.AddQuad({ { p0[0], p0[1], bottom }, { n0[0], n0[1], 0.0f } }, { { p1[0], p1[1], bottom }, { n1[0], n1[1], 0.0f } },
                { { p1[0], p1[1], top }, { n1[0], n1[1], 0.0f } }, { { p0[0], p0[1], top }, { n0[0], n0[1], 0.0f } });
        }

        Point2 centre{ 0.0f, 0.0f };
        for (const auto& point : outline)
            centre = { centre[0] + point[0] / static_cast<float>(count), centre[1] + point[1] / static_cast<float>(count) };
        const Vec3 up{ 0.0f, 0.0f, 1.0f };
        const Vec3 down{ 0.0f, 0.0f, -1.0f };
        for (std::size_t edge = 0; edge != count; ++edge)
        {
            const auto& p0 = outline[edge];
            const auto& p1 = outline[(edge + 1) % count];
            mesh.AddTriangle({ { centre[0], centre[1], top }, up }, { { p0[0], p0[1], top }, up }, { { p1[0], p1[1], top }, up });
            mesh.AddTriangle({ { centre[0], centre[1], bottom }, down }, { { p1[0], p1[1], bottom }, down }, { { p0[0], p0[1], bottom }, down });
        }
        return mesh;
    }

    Outline RoundedRectangle(float width, float height, float radius, std::size_t cornerSegments)
    {
        const auto r = std::clamp(radius, 0.0f, std::min(width, height) / 2.0f);
        const auto x = width / 2.0f - r;
        const auto y = height / 2.0f - r;
        Outline outline;
        const std::array<std::array<float, 3>, 4> corners{ { { x, -y, -pi / 2.0f }, { x, y, 0.0f }, { -x, y, pi / 2.0f }, { -x, -y, pi } } };
        for (const auto& [cx, cy, start] : corners)
        {
            auto arc = Arc(cx, cy, r, start, start + pi / 2.0f, std::max<std::size_t>(cornerSegments, 1));
            if (r == 0.0f)
                arc.resize(1);
            outline.insert(outline.end(), arc.begin(), arc.end());
        }
        return outline;
    }

    Outline Stadium(float length, float width, std::size_t endSegments)
    {
        const auto r = width / 2.0f;
        auto outline = Arc(0.0f, 0.0f, r, pi, 2.0f * pi, endSegments);
        const auto far = Arc(0.0f, length, r, 0.0f, pi, endSegments);
        outline.insert(outline.end(), far.begin(), far.end());
        return outline;
    }

    Outline RegularPolygon(std::size_t sides, float radius)
    {
        auto outline = Arc(0.0f, 0.0f, radius, 0.0f, 2.0f * pi, sides);
        outline.pop_back();
        return outline;
    }
}
