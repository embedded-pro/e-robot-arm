#include "tools/arm_console/CommandTracker.hpp"
#include <algorithm>
#include <array>
#include <string_view>

namespace arm_console
{
    namespace
    {
        constexpr std::array<std::string_view, 6> moveCommands{ "move", "move through", "move line", "go to", "jog joint", "jog tool" };

        std::optional<Answer> AnswerOf(arm_host::ArmEventKind kind)
        {
            using enum arm_host::ArmEventKind;
            switch (kind)
            {
                case accepted:
                case profileLoaded:
                    return Answer::accepted;
                case refused:
                case targetFailed:
                    return Answer::refused;
                default:
                    return std::nullopt;
            }
        }

        bool StopsWaitingForAState(arm::ArmState state)
        {
            return state == arm::ArmState::tripped || state == arm::ArmState::misconfigured || state == arm::ArmState::incompatible;
        }
    }

    void CommandTracker::Note(const arm_host::ArmEvent& event)
    {
        std::scoped_lock lock{ mutex };
        using enum arm_host::ArmEventKind;
        if (event.kind == moveDone || event.kind == moveAborted)
        {
            lastMoveEnd = event.kind == moveDone ? MoveEnd::done : MoveEnd::aborted;
            movePending = false;
        }
        else if (event.kind == stateChanged)
            state = event.state;
        else if (const auto answer = AnswerOf(event.kind); answer.has_value() && abandoned.erase(event.command) == 0)
        {
            answers.insert_or_assign(event.command, *answer);
            if (event.kind == accepted && std::ranges::find(moveCommands, event.commandName) != moveCommands.end())
                movePending = true;
        }
        changed.notify_all();
    }

    Answer CommandTracker::Await(uint32_t command, std::chrono::milliseconds timeout)
    {
        std::unique_lock lock{ mutex };
        if (!changed.wait_for(lock, timeout, [this, command]()
                {
                    return answers.contains(command);
                }))
        {
            abandoned.insert(command);
            return Answer::noAnswer;
        }

        return answers.extract(command).mapped();
    }

    std::optional<MoveEnd> CommandTracker::AwaitMove(std::chrono::milliseconds timeout)
    {
        std::unique_lock lock{ mutex };
        if (!changed.wait_for(lock, timeout, [this]()
                {
                    return !movePending;
                }))
            return std::nullopt;
        return lastMoveEnd.value_or(MoveEnd::done);
    }

    CommandTracker::StateWait CommandTracker::AwaitState(arm::ArmState wanted, std::chrono::milliseconds timeout)
    {
        std::unique_lock lock{ mutex };
        changed.wait_for(lock, timeout, [this, wanted]()
            {
                return state == wanted || (StopsWaitingForAState(state) && !StopsWaitingForAState(wanted));
            });
        return { state == wanted, state };
    }

    std::size_t CommandTracker::Held() const
    {
        std::scoped_lock lock{ mutex };
        return answers.size() + abandoned.size();
    }
}
