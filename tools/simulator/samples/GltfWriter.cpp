#include "tools/simulator/samples/GltfWriter.hpp"
#include <algorithm>
#include <bit>
#include <charconv>
#include <cstdint>
#include <format>
#include <limits>

namespace samples
{
    namespace
    {
        constexpr uint32_t arrayBuffer{ 34962 };
        constexpr uint32_t elementArrayBuffer{ 34963 };
        constexpr uint32_t floatComponent{ 5126 };
        constexpr uint32_t unsignedIntComponent{ 5125 };

        std::string Number(float value)
        {
            std::array<char, 32> text{};
            const auto result = std::to_chars(text.data(), text.data() + text.size(), value);
            return { text.data(), result.ptr };
        }

        std::string Vector(const std::array<float, 3>& value)
        {
            return "[" + Number(value[0]) + ", " + Number(value[1]) + ", " + Number(value[2]) + "]";
        }

        void AppendLittleEndian(std::vector<uint8_t>& buffer, uint32_t value)
        {
            for (uint32_t shift = 0; shift != 32; shift += 8)
                buffer.push_back(static_cast<uint8_t>(value >> shift));
        }

        void AppendFloats(std::vector<uint8_t>& buffer, const ui::stage::Vector3& value)
        {
            for (const auto component : { value.x, value.y, value.z })
                AppendLittleEndian(buffer, std::bit_cast<uint32_t>(component));
        }

        std::string Base64(const std::vector<uint8_t>& bytes)
        {
            constexpr std::string_view alphabet{ "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/" };
            std::string text;
            text.reserve((bytes.size() + 2) / 3 * 4);
            for (std::size_t at = 0; at < bytes.size(); at += 3)
            {
                const auto left = bytes.size() - at;
                const uint32_t group = (uint32_t{ bytes[at] } << 16) | (left > 1 ? uint32_t{ bytes[at + 1] } << 8 : 0) | (left > 2 ? uint32_t{ bytes[at + 2] } : 0);
                text += alphabet[(group >> 18) & 63];
                text += alphabet[(group >> 12) & 63];
                text += left > 1 ? alphabet[(group >> 6) & 63] : '=';
                text += left > 2 ? alphabet[group & 63] : '=';
            }
            return text;
        }

        struct Bounds
        {
            std::array<float, 3> min{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
            std::array<float, 3> max{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
        };

        Bounds PositionBounds(const ui::stage::ShadedMesh& mesh)
        {
            Bounds bounds;
            for (const auto& vertex : mesh.vertices)
            {
                const std::array<float, 3> position{ vertex.position.x, vertex.position.y, vertex.position.z };
                for (std::size_t axis = 0; axis != 3; ++axis)
                {
                    bounds.min[axis] = std::min(bounds.min[axis], position[axis]);
                    bounds.max[axis] = std::max(bounds.max[axis], position[axis]);
                }
            }
            return bounds;
        }

        class Document
        {
        public:
            void Add(const GltfPrimitive& primitive)
            {
                const auto& mesh = primitive.mesh;
                const auto index = materials.size();
                const auto bounds = PositionBounds(mesh);

                const auto positions = View(mesh.vertices.size() * 12, arrayBuffer, [&mesh](auto& buffer)
                    {
                        for (const auto& vertex : mesh.vertices)
                            AppendFloats(buffer, vertex.position);
                    });
                const auto normals = View(mesh.vertices.size() * 12, arrayBuffer, [&mesh](auto& buffer)
                    {
                        for (const auto& vertex : mesh.vertices)
                            AppendFloats(buffer, vertex.normal);
                    });
                const auto indices = View(mesh.indices.size() * 4, elementArrayBuffer, [&mesh](auto& buffer)
                    {
                        for (const auto vertex : mesh.indices)
                            AppendLittleEndian(buffer, vertex);
                    });

                const auto first = accessors.size();
                accessors.emplace_back(std::format(R"({}, "min": {}, "max": {} }})", Accessor(positions, floatComponent, mesh.vertices.size(), "VEC3"), Vector(bounds.min), Vector(bounds.max)));
                accessors.emplace_back(Accessor(normals, floatComponent, mesh.vertices.size(), "VEC3") + " }");
                accessors.emplace_back(Accessor(indices, unsignedIntComponent, mesh.indices.size(), "SCALAR") + " }");

                primitives.emplace_back(std::format(R"({{ "attributes": {{ "POSITION": {}, "NORMAL": {} }}, "indices": {}, "material": {}, "mode": 4 }})", first, first + 1, first + 2, index));
                materials.emplace_back(Material(primitive.material));
            }

            std::string Text(const std::string& nodeName) const
            {
                return std::format(R"({{
  "asset": {{ "version": "2.0", "generator": "e-robot-arm-sample-meshes" }},
  "scene": 0,
  "scenes": [ {{ "nodes": [ 0 ] }} ],
  "nodes": [ {{ "name": "{0}", "mesh": 0 }} ],
  "meshes": [ {{ "name": "{0}", "primitives": {1} }} ],
  "materials": {2},
  "accessors": {3},
  "bufferViews": {4},
  "buffers": [ {{ "byteLength": {5}, "uri": "data:application/octet-stream;base64,{6}" }} ]
}}
)",
                    nodeName, List(primitives), List(materials), List(accessors), List(views), buffer.size(), Base64(buffer));
            }

        private:
            template<class Fill>
            std::size_t View(std::size_t length, uint32_t target, Fill fill)
            {
                const auto offset = buffer.size();
                fill(buffer);
                views.emplace_back(std::format(R"({{ "buffer": 0, "byteOffset": {}, "byteLength": {}, "target": {} }})", offset, length, target));
                return views.size() - 1;
            }

            static std::string Accessor(std::size_t view, uint32_t component, std::size_t count, const char* type)
            {
                return std::format(R"({{ "bufferView": {}, "componentType": {}, "count": {}, "type": "{}")", view, component, count, type);
            }

            static std::string Material(const GltfMaterial& material)
            {
                const auto& c = material.linearColour;
                return std::format(R"({{ "name": "{}", "pbrMetallicRoughness": {{ "baseColorFactor": [{}, {}, {}, 1], "metallicFactor": {}, "roughnessFactor": {} }} }})", material.name, Number(c[0]),
                    Number(c[1]), Number(c[2]), Number(material.metallic), Number(material.roughness));
            }

            static std::string List(const std::vector<std::string>& items)
            {
                std::string text{ "[\n    " };
                for (std::size_t item = 0; item != items.size(); ++item)
                    text += (item == 0 ? "" : ",\n    ") + items[item];
                return text + "\n  ]";
            }

            std::vector<uint8_t> buffer;
            std::vector<std::string> views;
            std::vector<std::string> accessors;
            std::vector<std::string> primitives;
            std::vector<std::string> materials;
        };
    }

    std::string WriteGltf(const std::string& nodeName, const std::vector<GltfPrimitive>& primitives)
    {
        Document document;
        for (const auto& primitive : primitives)
            document.Add(primitive);
        return document.Text(nodeName);
    }
}
