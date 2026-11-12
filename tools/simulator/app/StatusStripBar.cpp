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
    }

    StatusStripBar::StatusStripBar(QWidget* parent)
        : QToolBar{ "Status", parent }
        , targets{ new QComboBox{ this } }
        , connection{ StatusLabel(this, "Connection") }
        , stage{ StatusLabel(this, "Arm stage") }
        , contracts{ StatusLabel(this, "Contracts") }
        , trip{ StatusLabel(this, "Trip") }
        , emergencyStop{ new QPushButton{ "EMERGENCY STOP\nEsc", this } }
    {
        setObjectName("statusStrip");
        setMovable(false);
        setFloatable(false);
        toggleViewAction()->setEnabled(false);

        targets->addItems({ "Simulated joints", "Virtual bus…", "Real arm…" });
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

        addWidget(new QLabel{ " Target ", this });
        addWidget(targets);
        addWidget(connection);
        addWidget(stage);
        addWidget(contracts);
        addWidget(trip);
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
        targets->setCurrentIndex(static_cast<int>(kind));
        targets->setEnabled(target.enabled);
        targets->setToolTip(target.enabled ? "Choose what the arm stack talks to" : ToQt(target.reason));

        connection->setText(ToQt(status.connection));
        StyleStatusLabel(*connection, status.connectionLevel);
        stage->setText(ToQt("Arm: " + status.stage));
        StyleStatusLabel(*stage, status.stageLevel);
        contracts->setText(ToQt(status.contracts));
        trip->setText(ToQt(status.trip));
        StyleStatusLabel(*trip, status.tripLevel);

        emergencyStop->setEnabled(emergency.enabled);
        emergencyStop->setToolTip(emergency.enabled ? "Emergency stop to every joint (Esc)" : ToQt(emergency.reason));
    }

    void StatusStripBar::ApplyTheme()
    {
        ui::backend::qt::StyleButton(*emergencyStop, ui::theme::ButtonRole::EmergencyStop);
    }
}
