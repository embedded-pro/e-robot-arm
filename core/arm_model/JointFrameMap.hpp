#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixOperations.hpp"
#include <array>
#include <cstdint>

namespace arm
{
    using JointVector = math::Vector<float, jointCount>;
    using JointMatrix = math::SquareMatrix<float, jointCount>;
    using JointValues = std::array<float, jointCount>;

    JointVector ToVector(const JointValues& values);
    JointValues ToValues(const JointVector& vector);

    template<class Quantity>
    JointVector ToVector(const std::array<Quantity, jointCount>& values)
    {
        JointVector vector{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            vector[joint] = values[joint].Value();
        return vector;
    }

    template<class Quantity>
    std::array<Quantity, jointCount> FromVector(const JointVector& vector)
    {
        std::array<Quantity, jointCount> values{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            values[joint] = Quantity{ vector[joint] };
        return values;
    }

    class JointFrameMap
    {
    public:
        using Signs = std::array<int8_t, jointCount>;
        using Offsets = std::array<float, jointCount>;

        JointFrameMap(const Signs& signs, const Offsets& offsets);

        JointVector Angles(const JointPositions& positions) const;
        JointPositions Positions(const JointVector& angles) const;

        JointVector Flip(const JointVector& vector) const;
        JointMatrix FlipColumns(const JointMatrix& matrix) const;
        JointMatrix Congruence(const JointMatrix& matrix) const;

        template<class Quantity>
        JointVector ToModel(const std::array<Quantity, jointCount>& values) const
        {
            return Flip(ToVector(values));
        }

        template<class Quantity>
        std::array<Quantity, jointCount> ToJoint(const JointVector& vector) const
        {
            return FromVector<Quantity>(Flip(vector));
        }

    private:
        JointVector signs;
        JointVector offsets;
    };
}
