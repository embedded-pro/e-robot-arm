#ifndef TOOLS_SIMULATOR_APP_EMERGENCY_STOP_KEY_HPP
#define TOOLS_SIMULATOR_APP_EMERGENCY_STOP_KEY_HPP

#include <QObject>
#include <functional>

namespace simulator
{
    class EmergencyStopKey
        : public QObject
    {
        Q_OBJECT

    public:
        EmergencyStopKey(QObject& application, std::function<void()> stop);
        EmergencyStopKey(const EmergencyStopKey& other) = delete;
        EmergencyStopKey& operator=(const EmergencyStopKey& other) = delete;
        ~EmergencyStopKey() override;

    protected:
        bool eventFilter(QObject* watched, QEvent* event) override;

    private:
        QObject& application;
        std::function<void()> stop;
        const QEvent* lastEvent{ nullptr };
    };
}

#endif
