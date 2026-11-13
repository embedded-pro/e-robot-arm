#ifndef CORE_ARM_MODEL_ARM_MASS_HPP
#define CORE_ARM_MODEL_ARM_MASS_HPP

#include <array>

namespace arm
{
    struct MassProperties
    {
        float mass{ 0.0f };
        std::array<float, 3> centre{};
        std::array<float, 6> inertia{};
    };

    struct ArmMass
    {
        std::array<MassProperties, 6> links{};
        MassProperties payload{};
    };

    inline ArmMass ReferenceMass()
    {
        return ArmMass{ { { { 1.2f, { 0.01f, 0.0f, 0.08f }, { 0.003f, 0.003f, 0.0015f, 0.0f, 0.0f, 0.0f } },
                            { 1.0f, { 0.0f, 0.0f, 0.11f }, { 0.0052f, 0.0052f, 0.00045f, 0.0f, 0.0f, 0.0f } },
                            { 0.8f, { 0.0f, 0.0f, 0.10f }, { 0.0042f, 0.0042f, 0.00036f, 0.0f, 0.0f, 0.0f } },
                            { 0.3f, { 0.0f, 0.0f, -0.03f }, { 0.0002f, 0.0002f, 0.0001f, 0.0f, 0.0f, 0.0f } },
                            { 0.25f, { 0.0f, 0.0f, 0.02f }, { 0.0001f, 0.0001f, 0.00005f, 0.0f, 0.0f, 0.0f } },
                            { 0.1f, { 0.0f, 0.0f, 0.05f }, { 0.00003f, 0.00003f, 0.00002f, 0.0f, 0.0f, 0.0f } } } },
            MassProperties{} };
    }
}

#endif
