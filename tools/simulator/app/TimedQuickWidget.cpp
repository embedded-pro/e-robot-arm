#include "tools/simulator/app/TimedQuickWidget.hpp"
#include <QElapsedTimer>
#include <QQuickWindow>

namespace simulator
{
    TimedQuickWidget::TimedQuickWidget(const std::function<void(std::chrono::nanoseconds duration)>& onFrame, QWidget* parent)
        : QQuickWidget{ parent }
        , onFrame{ onFrame }
    {
        connect(quickWindow(), &QQuickWindow::afterRendering, this, [this]()
            {
                rendered = true;
            });
    }

    bool TimedQuickWidget::event(QEvent* event)
    {
        rendered = false;
        QElapsedTimer handling;
        handling.start();
        const auto handled = QQuickWidget::event(event);
        if (rendered)
            onFrame(std::chrono::nanoseconds{ handling.nsecsElapsed() });
        rendered = false;
        return handled;
    }
}
