#include "core/arm_model/JointFrameMap.hpp"

namespace arm
{
    namespace
    {
        JointVector SignVector(const JointFrameMap::Signs& signs)
        {
            JointVector vector{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                vector[joint] = static_cast<float>(signs[joint]);
            return vector;
        }

        JointVector OffsetVector(const JointFrameMap::Offsets& offsets)
        {
            JointVector vector{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                vector[joint] = offsets[joint];
            return vector;
        }
    }

    JointFrameMap::JointFrameMap(const Signs& signs, const Offsets& offsets)
        : signs(SignVector(signs))
        , offsets(OffsetVector(offsets))
    {}

    JointVector JointFrameMap::Angles(const JointPositions& positions) const
    {
        return Flip(ToVector(positions)) - offsets;
    }

    JointPositions JointFrameMap::Positions(const JointVector& angles) const
    {
        return FromVector<foc::Radians>(Flip(angles + offsets));
    }

    JointVector JointFrameMap::Flip(const JointVector& vector) const
    {
        JointVector flipped{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            flipped[joint] = signs[joint] * vector[joint];
        return flipped;
    }

    JointMatrix JointFrameMap::FlipColumns(const JointMatrix& matrix) const
    {
        auto flipped = matrix;
        for (std::size_t row = 0; row != jointCount; ++row)
            for (std::size_t column = 0; column != jointCount; ++column)
                flipped.at(row, column) *= signs[column];
        return flipped;
    }

    JointMatrix JointFrameMap::Congruence(const JointMatrix& matrix) const
    {
        auto flipped = FlipColumns(matrix);
        for (std::size_t row = 0; row != jointCount; ++row)
            for (std::size_t column = 0; column != jointCount; ++column)
                flipped.at(row, column) *= signs[row];
        return flipped;
    }
}
