#include "tools/arm_console/ConsoleOptions.hpp"
#include <algorithm>
#include <charconv>
#include <sstream>
#include <string_view>

namespace arm_console
{
    namespace
    {
        template<class T>
        std::optional<T> Number(std::string_view text)
        {
            T value{};
            const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
            if (error != std::errc{} || end != text.data() + text.size())
                return std::nullopt;
            return value;
        }

        std::optional<arm::JointNodeIds> Nodes(const std::string& text)
        {
            arm::JointNodeIds nodes{};
            std::istringstream stream{ text };
            std::string item;
            std::size_t count = 0;
            while (std::getline(stream, item, ','))
            {
                const auto node = Number<uint16_t>(item);
                if (count == nodes.size() || !node.has_value() || *node == 0 || *node > 0xfff || std::ranges::find(nodes.begin(), nodes.begin() + count, *node) != nodes.begin() + count)
                    return std::nullopt;
                nodes[count++] = *node;
            }

            if (count != nodes.size())
                return std::nullopt;
            return nodes;
        }

        bool Apply(ConsoleOptions& options, std::string_view name, const std::string& value)
        {
            if (name == "--bus")
                options.bus = value;
            else if (name == "--bitrate")
            {
                const auto bitrate = Number<uint32_t>(value);
                if (!bitrate.has_value())
                    return false;
                options.bitrate = *bitrate;
            }
            else if (name == "--nodes")
            {
                const auto nodes = Nodes(value);
                if (!nodes.has_value())
                    return false;
                options.nodes = *nodes;
            }
            else
                return false;

            return true;
        }
    }

    std::optional<ConsoleOptions> ParseOptions(const std::vector<std::string>& arguments)
    {
        ConsoleOptions options;
        for (std::size_t index = 1; index < arguments.size(); index += 2)
            if (index + 1 == arguments.size() || !Apply(options, arguments[index], arguments[index + 1]))
                return std::nullopt;

        return options;
    }

    std::string OptionsUsage()
    {
        return "usage: e-robot-arm-console [--bus virtual:<name>|<socketcan interface>|<serial port>] [--bitrate <bits/s>] [--nodes n1,n2,n3,n4,n5,n6]";
    }
}
