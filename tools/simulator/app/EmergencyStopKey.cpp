#include "tools/simulator/app/EmergencyStopKey.hpp"
#include <QKeyEvent>

namespace simulator
{
    EmergencyStopKey::EmergencyStopKey(QObject& application, std::function<void()> stop)
        : application(application)
        , stop(std::move(stop))
    {
        application.installEventFilter(this);
    }

    EmergencyStopKey::~EmergencyStopKey()
    {
        application.removeEventFilter(this);
    }

    bool EmergencyStopKey::eventFilter(QObject* watched, QEvent* event)
    {
        if (event->type() == QEvent::KeyPress && event != lastEvent)
        {
            const auto* key = static_cast<const QKeyEvent*>(event);
            if (key->key() == Qt::Key_Escape && !key->isAutoRepeat())
            {
                lastEvent = event;
                stop();
            }
        }

        return QObject::eventFilter(watched, event);
    }
}
