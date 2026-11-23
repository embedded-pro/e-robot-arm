#pragma once

#include "tools/arm_host/ArmEvent.hpp"
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>

namespace arm_console
{
    enum class Answer : uint8_t
    {
        accepted,
        refused,
        noAnswer
    };

    enum class MoveEnd : uint8_t
    {
        done,
        aborted
    };

    class CommandTracker
    {
    public:
        struct StateWait
        {
            bool reached;
            arm::ArmState state;
        };

        void Note(const arm_host::ArmEvent& event);
        Answer Await(uint32_t command, std::chrono::milliseconds timeout);
        std::optional<MoveEnd> AwaitMove(std::chrono::milliseconds timeout);
        StateWait AwaitState(arm::ArmState wanted, std::chrono::milliseconds timeout);
        std::size_t Held() const;

    private:
        mutable std::mutex mutex;
        std::condition_variable changed;
        std::map<uint32_t, Answer> answers;
        std::set<uint32_t> abandoned;
        bool movePending{ false };
        std::optional<MoveEnd> lastMoveEnd;
        arm::ArmState state{ arm::ArmState::unpowered };
    };
}
