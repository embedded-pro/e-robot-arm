#include "tools/simulator/view_models/RenderQuality.hpp"
#include "gmock/gmock.h"

namespace
{
    using simulator::ChooseRenderQuality;
    using simulator::IsSoftwareRenderer;
    using simulator::RendererInfo;
    using simulator::RenderQuality;
    using simulator::RenderQualitySetting;
}

TEST(RenderQualityTest, a_cpu_device_is_a_software_renderer)
{
    EXPECT_TRUE(IsSoftwareRenderer(RendererInfo{ "Some Adapter", true }));
}

TEST(RenderQualityTest, known_software_renderers_are_recognised_by_name)
{
    EXPECT_TRUE(IsSoftwareRenderer(RendererInfo{ "Microsoft Basic Render Driver", false }));
    EXPECT_TRUE(IsSoftwareRenderer(RendererInfo{ "llvmpipe (LLVM 15.0.7, 256 bits)", false }));
    EXPECT_TRUE(IsSoftwareRenderer(RendererInfo{ "softpipe", false }));
    EXPECT_TRUE(IsSoftwareRenderer(RendererInfo{ "Google SwiftShader", false }));
}

TEST(RenderQualityTest, a_gpu_is_not_a_software_renderer)
{
    EXPECT_FALSE(IsSoftwareRenderer(RendererInfo{ "NVIDIA GeForce RTX 3060", false }));
}

TEST(RenderQualityTest, automatic_picks_low_on_a_software_renderer)
{
    EXPECT_EQ(RenderQuality::low, ChooseRenderQuality(RenderQualitySetting::automatic, RendererInfo{ "Microsoft Basic Render Driver", true }));
}

TEST(RenderQualityTest, automatic_picks_high_on_a_gpu_or_an_unknown_renderer)
{
    EXPECT_EQ(RenderQuality::high, ChooseRenderQuality(RenderQualitySetting::automatic, RendererInfo{ "NVIDIA GeForce RTX 3060", false }));
    EXPECT_EQ(RenderQuality::high, ChooseRenderQuality(RenderQualitySetting::automatic, std::nullopt));
}

TEST(RenderQualityTest, an_explicit_setting_wins_over_the_renderer)
{
    EXPECT_EQ(RenderQuality::high, ChooseRenderQuality(RenderQualitySetting::high, RendererInfo{ "llvmpipe", true }));
    EXPECT_EQ(RenderQuality::low, ChooseRenderQuality(RenderQualitySetting::low, RendererInfo{ "NVIDIA GeForce RTX 3060", false }));
}

TEST(RenderQualityTest, the_announcement_names_the_renderer_and_the_menu)
{
    EXPECT_EQ("Software renderer Microsoft Basic Render Driver: 3-D view at low quality (View → Render Quality changes it)",
        simulator::SoftwareRendererNotice(RendererInfo{ "Microsoft Basic Render Driver", true }));
}

TEST(RenderQualityTest, a_stored_setting_outside_the_range_reads_as_automatic)
{
    EXPECT_EQ(RenderQualitySetting::low, simulator::RenderQualitySettingFrom(2));
    EXPECT_EQ(RenderQualitySetting::automatic, simulator::RenderQualitySettingFrom(7));
    EXPECT_EQ(RenderQualitySetting::automatic, simulator::RenderQualitySettingFrom(-1));
}
