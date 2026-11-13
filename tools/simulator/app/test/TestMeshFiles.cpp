#include "tools/simulator/app/MeshFiles.hpp"
#include "gmock/gmock.h"
#include <QFile>
#include <QTemporaryDir>
#include <QtEndian>

namespace
{
    const QString box{ QStringLiteral(MESH_TEST_DATA "/box.gltf") };

    QByteArray Gltf(int count, int mode)
    {
        return QString{ R"({ "asset": { "version": "2.0" }, "accessors": [ { "count": %1 } ],
            "meshes": [ { "primitives": [ { "attributes": { "POSITION": 0 }, "mode": %2 } ] } ] })" }
            .arg(count)
            .arg(mode)
            .toUtf8();
    }

    QByteArray Glb(const QByteArray& json)
    {
        auto padded = json;
        while (padded.size() % 4 != 0)
            padded.append(' ');
        QByteArray glb(20, '\0');
        qToLittleEndian<quint32>(0x46546C67, glb.data());
        qToLittleEndian<quint32>(2, glb.data() + 4);
        qToLittleEndian<quint32>(static_cast<quint32>(20 + padded.size()), glb.data() + 8);
        qToLittleEndian<quint32>(static_cast<quint32>(padded.size()), glb.data() + 12);
        qToLittleEndian<quint32>(0x4E4F534A, glb.data() + 16);
        return glb + padded;
    }

    class MeshFilesTest
        : public testing::Test
    {
    public:
        QString Write(const QString& name, const QByteArray& contents)
        {
            const auto path = directory.filePath(name);
            QFile file{ path };
            EXPECT_TRUE(file.open(QIODevice::WriteOnly));
            file.write(contents);
            return path;
        }

        QTemporaryDir directory;
    };
}

TEST_F(MeshFilesTest, counts_the_triangles_of_text_and_binary_gltf)
{
    QFile file{ box };
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    EXPECT_EQ(12u, simulator::CountTriangles(file.readAll()));

    EXPECT_EQ(100u, simulator::CountTriangles(Glb(Gltf(300, 4))));
    EXPECT_EQ(98u, simulator::CountTriangles(Gltf(100, 5)));
    EXPECT_EQ(0u, simulator::CountTriangles(Gltf(100, 1)));
    EXPECT_FALSE(simulator::CountTriangles("solid stl").has_value());
}

TEST_F(MeshFilesTest, refuses_a_missing_foreign_or_heavy_mesh_with_the_reason)
{
    EXPECT_TRUE(simulator::CheckMesh(directory.filePath("absent.glb")).problem.startsWith("cannot open"));
    EXPECT_TRUE(simulator::CheckMesh(Write("arm.glb", "solid stl")).problem.endsWith("is not a glTF file"));

    const auto heavy = simulator::CheckMesh(Write("heavy.gltf", Gltf(3 * 50001, 4)));
    EXPECT_TRUE(heavy.url.isEmpty());
    EXPECT_TRUE(heavy.problem.endsWith("has 50001 triangles, more than 50000")) << heavy.problem.toStdString();
}

TEST_F(MeshFilesTest, resolves_names_relative_to_the_profile_directory)
{
    QFile::copy(box, directory.filePath("upper.gltf"));
    const simulator::MeshNames names{ "", "", "upper.gltf", "", "", "", "" };

    const auto files = simulator::ResolveMeshes(names, directory.path());

    EXPECT_EQ(QUrl::fromLocalFile(directory.filePath("upper.gltf")), files[2].url);
    EXPECT_TRUE(files[2].problem.isEmpty());
    EXPECT_TRUE(files[1].url.isEmpty());
    EXPECT_TRUE(files[1].problem.isEmpty());
    EXPECT_EQ("Base", simulator::MeshNodeName(0));
    EXPECT_EQ("Link 6", simulator::MeshNodeName(6));
}
