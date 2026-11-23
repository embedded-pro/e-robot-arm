#include "tools/simulator/samples/SampleArm.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: e-robot-arm-sample-meshes <directory>\n";
        return 2;
    }

    const std::filesystem::path directory{ argv[1] };
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
    {
        std::cerr << "cannot create " << directory.string() << ": " << error.message() << "\n";
        return 1;
    }
    for (const auto& file : samples::SampleArmFiles())
    {
        std::ofstream stream{ directory / file.name, std::ios::binary | std::ios::trunc };
        stream << file.text;
        if (!stream)
        {
            std::cerr << "cannot write " << (directory / file.name).string() << "\n";
            return 1;
        }
    }
    return 0;
}
