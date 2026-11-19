#include "tools/simulator/app/StatusStripBar.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>

namespace simulator
{
    namespace
    {
        QLabel* StatusLabel(QWidget* parent, const char* accessibleName)
        {
            auto* label = new QLabel{ parent };
            label->setAccessibleName(QString::fromUtf8(accessibleName));
            label->setContentsMargins(8, 0, 8, 0);
            return label;
        }

        int ChoiceOf(arm_host::TargetKind kind)
        {
            switch (kind)
            {
                case arm_host::TargetKind::simulated:
                    return StatusStripBar::simulatedJoints;
                case arm_host::TargetKind::virtualArm:
                    return StatusStripBar::virtualArm;
                case arm_host::TargetKind::virtualBus:
                    return StatusStripBar::virtualBus;
                case arm_host::TargetKind::real:
                    return StatusStripBar::realArm;
            }
            return StatusStripBar::simulatedJoints;
        }
    }

    StatusStripBar::StatusStripBar(QWidget* parent)
        : QToolBar{ "Status", parent }
        , targets{ new QComboBox{ this } }
        , connection{ StatusLabel(this, "Connection") }
        , stage{ StatusLabel(this, "Arm state") }
        , contracts{ StatusLabel(this, "Contracts") }
        , stop{ StatusLabel(this, "Stop") }
        , torqueLimited{ StatusLabel(this, "Torque limited") }
        , bus{ StatusLabel(this, "Bus load") }
        , overruns{ StatusLabel(this, "Overruns") }
        , limits{ StatusLabel(this, "Limits") }
        , emergencyStop{ new QPushButton{ "EMERGENCY STOP\nEsc", this } }
    {
        setObjectName("statusStrip");
        setMovable(false);
        setFloatable(false);
        toggleViewAction()->setEnabled(false);

        targets->addItems({ "Simulated joints", "Virtual arm…", "Virtual bus…", "Real arm…" });
        targets->setAccessibleName("Target");
        targets->setToolTip("Choose what the arm stack talks to");

        auto* spacer = new QWidget{ this };
        spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        emergencyStop->setAccessibleName("Emergency stop");
        emergencyStop->setMinimumSize(200, 56);
        auto font = emergencyStop->font();
        font.setBold(true);
        font.setPointSizeF(font.pointSizeF() * 1.25);
        emergencyStop->setFont(font);
        emergencyStop->setFocusPolicy(Qt::NoFocus);
        overruns->setToolTip("Control-loop phases that started more than a quarter period late");

        addWidget(new QLabel{ " Target ", this });
        addWidget(targets);
        addWidget(connection);
        addWidget(stage);
        addWidget(contracts);
        addWidget(stop);
        torqueLimitedItem = addWidget(torqueLimited);
        torqueLimited->setToolTip("Joints whose torque was clamped to its limit in the last half second");
        torqueLimitedItem->setVisible(false);
        bus->setToolTip("CAN frames sent and received in the last second over the bitrate, without bit stuffing");
        addWidget(bus);
        addWidget(overruns);
        limitsItem = addWidget(limits);
        limits->setToolTip("The limits the real arm runs under: Arm → Commissioning Limits");
        limitsItem->setVisible(false);
        addWidget(spacer);
        addWidget(emergencyStop);

        connect(targets, &QComboBox::activated, this, &StatusStripBar::targetChosen);
        connect(emergencyStop, &QPushButton::pressed, this, &StatusStripBar::emergencyStopPressed);
        ApplyTheme();
    }

    void StatusStripBar::Update(const StatusStrip& status, const ControlState& emergency, const ControlState& target, arm_host::TargetKind kind)
    {
        using ui::backend::qt::StyleStatusLabel;
        using ui::backend::qt::ToQt;

        const QSignalBlocker blocker{ targets };
        targets->setCurrentIndex(ChoiceOf(kind));
        targets->setEnabled(target.enabled);
        targets->setToolTip(target.enabled ? "Choose what the arm stack talks to" : ToQt(target.reason));

        connection->setText(ToQt(status.connection));
        StyleStatusLabel(*connection, status.connectionLevel);
        stage->setText(ToQt("Arm: " + status.stage));
        StyleStatusLabel(*stage, status.stageLevel);
        contracts->setText(ToQt(status.contracts));
        stop->setText(ToQt(status.stop));
        stop->setToolTip(ToQt(status.stopDetail));
        StyleStatusLabel(*stop, status.stopLevel);
        torqueLimited->setText(ToQt(status.torqueLimited));
        StyleStatusLabel(*torqueLimited, status.torqueLimitedLevel);
        torqueLimitedItem->setVisible(!status.torqueLimited.empty());
        bus->setText(ToQt(status.bus));
        StyleStatusLabel(*bus, status.busLevel);
        overruns->setText(ToQt(status.overruns));
        StyleStatusLabel(*overruns, status.overrunLevel);
        limits->setText(ToQt(status.limits));
        StyleStatusLabel(*limits, status.limitsLevel);
        limitsItem->setVisible(!status.limits.empty());

        emergencyStop->setEnabled(emergency.enabled);
        emergencyStop->setToolTip(emergency.enabled ? "Emergency stop to every joint (Esc)" : ToQt(emergency.reason));
    }

    void StatusStripBar::ApplyTheme()
    {
        ui::backend::qt::StyleButton(*emergencyStop, ui::theme::ButtonRole::EmergencyStop);
    }
}
