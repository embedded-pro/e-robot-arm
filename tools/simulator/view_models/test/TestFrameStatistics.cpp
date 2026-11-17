#include "tools/simulator/view_models/FrameStatistics.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace std::chrono_literals;
    using ui::theme::StatusLevel;

    class FrameStatisticsTest
        : public testing::Test
    {
    public:
        void Present(int frames, std::chrono::nanoseconds render)
        {
            for (int frame = 0; frame != frames; ++frame)
                statistics.AddRender(render);
        }

        simulator::FrameStatistics statistics{ 0ns };
    };
}

TEST_F(FrameStatisticsTest, summarises_each_second_of_frames_once_it_has_passed)
{
    EXPECT_EQ("3-D — · GUI —", statistics.Text());

    Present(58, 6ms);
    for (int frame = 0; frame != 9; ++frame)
        statistics.AddGuiFrame(2ms);
    statistics.AddGuiFrame(4ms);

    EXPECT_FALSE(statistics.Advance(999ms));
    EXPECT_TRUE(statistics.Advance(1000ms));
    EXPECT_EQ("3-D 58 fps · render 6.0 ms · GUI 2.2 ms (max 4.0 ms)", statistics.Text());
    EXPECT_EQ(StatusLevel::Neutral, statistics.Level());

    statistics.AddGuiFrame(1ms);
    EXPECT_TRUE(statistics.Advance(2000ms));
    EXPECT_EQ("3-D idle · GUI 1.0 ms (max 1.0 ms)", statistics.Text());
    EXPECT_EQ(StatusLevel::Neutral, statistics.Level());
}

TEST_F(FrameStatisticsTest, warns_below_30_fps_or_when_a_gui_frame_overruns_a_60_fps_frame)
{
    Present(19, 40ms);
    statistics.AddGuiFrame(2ms);
    statistics.Advance(2000ms);

    EXPECT_EQ("3-D 9.5 fps · render 40.0 ms · GUI 2.0 ms (max 2.0 ms)", statistics.Text());
    EXPECT_EQ(StatusLevel::Warning, statistics.Level());

    Present(60, 5ms);
    statistics.AddGuiFrame(17ms);
    statistics.Advance(3000ms);

    EXPECT_EQ(StatusLevel::Warning, statistics.Level());

    Present(60, 5ms);
    statistics.AddGuiFrame(16ms);
    statistics.Advance(4000ms);

    EXPECT_EQ(StatusLevel::Neutral, statistics.Level());
}
