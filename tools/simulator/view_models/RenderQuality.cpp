#include "tools/simulator/view_models/RenderQuality.hpp"
#include <algorithm>
#include <array>

namespace simulator
{
    bool IsSoftwareRenderer(const RendererInfo& renderer)
    {
        constexpr std::array<std::string_view, 4> names{ "Microsoft Basic Render Driver", "llvmpipe", "softpipe", "SwiftShader" };
        return renderer.cpuDevice || std::ranges::any_of(names, [&renderer](std::string_view name)
                                         {
                                             return renderer.deviceName.find(name) != std::string::npos;
                                         });
    }

    RenderQuality ChooseRenderQuality(RenderQualitySetting setting, const std::optional<RendererInfo>& renderer)
    {
        switch (setting)
        {
            case RenderQualitySetting::high:
                return RenderQuality::high;
            case RenderQualitySetting::low:
                return RenderQuality::low;
            default:
                return renderer && IsSoftwareRenderer(*renderer) ? RenderQuality::low : RenderQuality::high;
        }
    }

    std::string SoftwareRendererNotice(const RendererInfo& renderer)
    {
        return "Software renderer " + renderer.deviceName + ": 3-D view at low quality (View → Render Quality changes it)";
    }

    RenderQualitySetting RenderQualitySettingFrom(int stored)
    {
        return stored == 1 ? RenderQualitySetting::high : stored == 2 ? RenderQualitySetting::low
                                                                      : RenderQualitySetting::automatic;
    }
}
