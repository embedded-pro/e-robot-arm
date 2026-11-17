#ifndef TOOLS_SIMULATOR_APP_TIMED_QUICK_WIDGET_HPP
#define TOOLS_SIMULATOR_APP_TIMED_QUICK_WIDGET_HPP

#include <QQuickWidget>
#include <chrono>
#include <functional>

namespace simulator
{
    class TimedQuickWidget
        : public QQuickWidget
    {
        Q_OBJECT

    public:
        TimedQuickWidget(const std::function<void(std::chrono::nanoseconds duration)>& onFrame, QWidget* parent);

    protected:
        bool event(QEvent* event) override;

    private:
        std::function<void(std::chrono::nanoseconds duration)> onFrame;
        bool rendered{ false };
    };
}

#endif
