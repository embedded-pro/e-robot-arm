#include "core/arm_controllers/implementations/SmoothedJointPosition.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <chrono>
#include <utility>

namespace arm
{
    namespace
    {
        float Weight(infra::Duration period, infra::Duration timeConstant)
        {
            really_assert(period > infra::Duration::zero() && timeConstant >= infra::Duration::zero());
            const auto t = std::chrono::duration<float>(period).count();
            const auto tau = std::chrono::duration<float>(timeConstant).count();
            return t / (tau + t);
        }

        template<std::size_t... Joint>
        auto Filters(float weight, std::index_sequence<Joint...>)
        {
            return std::array{ ((void)Joint, filters::passive::ExponentialMovingAverage<float>{ weight })... };
        }
    }

    SmoothedJointPosition::SmoothedJointPosition(infra::Duration period, infra::Duration timeConstant)
        : filters(Filters(Weight(period, timeConstant), std::make_index_sequence<jointCount>{}))
    {}

    bool SmoothedJointPosition::BumplessTransfer() const
    {
        return true;
    }

    void SmoothedJointPosition::Reset(const MeasuredJoints& measured)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            filters[joint].Reset(measured.positions[joint].Value());
    }

    JointSetpoints SmoothedJointPosition::Step(const MeasuredJoints&, const JointPositions& reference)
    {
        JointSetpoints setpoints;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            setpoints.positions[joint] = foc::Radians{ filters[joint].Filter(reference[joint].Value()) };
        return setpoints;
    }
}
