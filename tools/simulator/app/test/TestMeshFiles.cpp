#include "tools/simulator/app/MeshFiles.hpp"
#include "gmock/gmock.h"
#include <QFile>
#include <QTemporaryDir>

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

TEST_F(MeshFilesTest, holds_each_mesh_to_the_simulator_budget)
{
    Write("light.gltf", Gltf(3 * 50000, 4));
    Write("heavy.gltf", Gltf(3 * 50001, 4));
    const simulator::MeshNames names{ "light.gltf", "heavy.gltf", "", "", "", "", "" };

    const auto files = simulator::ResolveMeshes(names, directory.path());

    EXPECT_FALSE(files[0].url.isEmpty());
    EXPECT_TRUE(files[1].url.isEmpty());
    EXPECT_TRUE(files[1].problem.endsWith("has 50001 triangles, more than 50000")) << files[1].problem.toStdString();
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
