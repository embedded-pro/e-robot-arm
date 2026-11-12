#ifndef TOOLS_SIMULATOR_APP_STATUS_STRIP_BAR_HPP
#define TOOLS_SIMULATOR_APP_STATUS_STRIP_BAR_HPP

#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include <QToolBar>

class QComboBox;
class QLabel;
class QPushButton;

namespace simulator
{
    class StatusStripBar
        : public QToolBar
    {
        Q_OBJECT

    public:
        enum TargetChoice
        {
            simulatedJoints,
            virtualBus,
            realArm
        };

        explicit StatusStripBar(QWidget* parent);

        void Update(const StatusStrip& status, const ControlState& emergencyStop, const ControlState& target, arm_host::TargetKind kind);
        void ApplyTheme();

    signals:
        void targetChosen(int choice);
        void emergencyStopPressed();

    private:
        QComboBox* targets;
        QLabel* connection;
        QLabel* stage;
        QLabel* contracts;
        QLabel* trip;
        QPushButton* emergencyStop;
    };
}

#endif
