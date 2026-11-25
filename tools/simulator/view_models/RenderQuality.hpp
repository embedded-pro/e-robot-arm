#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace simulator
{
    enum class RenderQualitySetting
    {
        automatic,
        high,
        low
    };

    enum class RenderQuality
    {
        high,
        low
    };

    struct RendererInfo
    {
        std::string deviceName;
        bool cpuDevice{ false };
    };

    bool IsSoftwareRenderer(const RendererInfo& renderer);
    RenderQuality ChooseRenderQuality(RenderQualitySetting setting, const std::optional<RendererInfo>& renderer);
    std::string SoftwareRendererNotice(const RendererInfo& renderer);
    RenderQualitySetting RenderQualitySettingFrom(int stored);
}
