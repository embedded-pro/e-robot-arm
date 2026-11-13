#include "tools/simulator/app/Panels.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QComboBox>
#include <QFile>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>
#include <algorithm>

namespace simulator
{
    FormPanel::FormPanel(QWidget* parent)
        : QWidget{ parent }
        , form{ new ui::backend::qt::QtFormView{ this } }
        , hint{ new QLabel{ this } }
    {
        hint->setWordWrap(true);
        hint->setAccessibleName("Why controls are disabled");
        ui::backend::qt::StyleStatusLabel(*hint, ui::theme::StatusLevel::Neutral);

        auto* layout = new QVBoxLayout{ this };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSpacing(8);
        layout->addWidget(form);
        layout->addWidget(hint);
        layout->addStretch();
    }

    ui::backend::qt::QtFormView& FormPanel::Form()
    {
        return *form;
    }

    void FormPanel::SetHint(const std::string& text)
    {
        const auto value = QString::fromStdString(text);
        if (hint->text() != value)
            hint->setText(value);
    }

    ConfigurationPanel::ConfigurationPanel(QWidget* parent)
        : FormPanel{ parent }
        , state{ new QLabel{ this } }
    {
        state->setWordWrap(true);
        state->setAccessibleName("Motor controller state");
        ui::backend::qt::StyleStatusLabel(*state, level);
        static_cast<QVBoxLayout*>(layout())->insertWidget(0, state);
    }

    void ConfigurationPanel::SetState(const std::string& text, ui::theme::StatusLevel newLevel)
    {
        const auto value = QString::fromStdString(text);
        if (state->text() != value)
            state->setText(value);
        if (newLevel != level)
        {
            level = newLevel;
            ui::backend::qt::StyleStatusLabel(*state, level);
        }
    }

    void ConfigurationPanel::ApplyTheme()
    {
        ui::backend::qt::StyleStatusLabel(*state, level);
    }

    ProfilePanel::ProfilePanel(QWidget* parent)
        : QWidget{ parent }
        , name{ new QLabel{ this } }
        , file{ new QLabel{ this } }
        , summary{ new QLabel{ this } }
        , table{ new QTableWidget{ static_cast<int>(arm::jointCount), static_cast<int>(profileColumns.size()), this } }
    {
        name->setAccessibleName("Profile name");
        file->setAccessibleName("Profile file");
        file->setTextInteractionFlags(Qt::TextSelectableByMouse);
        summary->setAccessibleName("Profile timing and enable mode");
        summary->setWordWrap(true);
        auto font = name->font();
        font.setBold(true);
        name->setFont(font);

        QStringList headings;
        for (const auto heading : profileColumns)
            headings << ui::backend::qt::ToQt(heading);
        table->setObjectName("profileTable");
        table->setAccessibleName("Profile per joint");
        table->setHorizontalHeaderLabels(headings);
        table->verticalHeader()->setVisible(false);
        table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        table->horizontalHeader()->setStretchLastSection(true);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setSelectionMode(QAbstractItemView::NoSelection);
        table->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));

        auto* header = new QHBoxLayout{};
        header->addWidget(name);
        header->addStretch();
        header->addWidget(file);

        auto* layout = new QVBoxLayout{ this };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSpacing(8);
        layout->addLayout(header);
        layout->addWidget(summary);
        layout->addWidget(table, 1);
    }

    void ProfilePanel::Update(const ProfileView& view, const QString& path)
    {
        if (view == shown && path == shownPath)
            return;

        shown = view;
        shownPath = path;
        name->setText(ui::backend::qt::ToQt(view.name));
        file->setText(path.isEmpty() ? QString{ "built-in profile" } : path);
        summary->setText(ui::backend::qt::ToQt(view.summary));

        for (int row = 0; row != table->rowCount(); ++row)
        {
            const auto& joint = view.rows[static_cast<std::size_t>(row)];
            const std::array<const std::string*, profileColumns.size()> cells{ &joint.joint, &joint.node, &joint.ratio, &joint.direction, &joint.offset, &joint.lower,
                &joint.upper, &joint.speed, &joint.acceleration, &joint.controller, &joint.motor };
            for (int column = 0; column != table->columnCount(); ++column)
            {
                auto* cell = new QTableWidgetItem{ ui::backend::qt::ToQt(*cells[static_cast<std::size_t>(column)]) };
                cell->setTextAlignment(column == 0 || column == 3 ? Qt::AlignCenter : (column >= 9 ? Qt::AlignLeft | Qt::AlignVCenter : Qt::AlignRight | Qt::AlignVCenter));
                table->setItem(row, column, cell);
            }
        }
    }

    ScopePanel::ScopePanel(QWidget* parent)
        : QWidget{ parent }
        , joints{ new QComboBox{ this } }
        , runHold{ new QPushButton{ "Hold", this } }
        , plot{ new ui::backend::qt::QtPaintedWidget{ scope, this } }
    {
        for (std::size_t index = 1; index <= arm::jointCount; ++index)
            joints->addItem(QString{ "Joint %1" }.arg(index));
        joints->setAccessibleName("Scope joint");
        runHold->setAccessibleName("Run or hold the scope");
        runHold->setCheckable(true);
        plot->SetBackgroundRole(ui::theme::ColorRole::ScopeBackground);
        plot->setMinimumHeight(160);

        auto* controls = new QHBoxLayout;
        controls->addWidget(new QLabel{ "Joint", this });
        controls->addWidget(joints);
        controls->addWidget(runHold);
        controls->addStretch();

        auto* layout = new QVBoxLayout{ this };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->addLayout(controls);
        layout->addWidget(plot, 1);

        connect(joints, &QComboBox::currentIndexChanged, this, [this](int index)
            {
                joint = static_cast<std::size_t>(index);
                scope.Clear();
            });
        connect(runHold, &QPushButton::toggled, this, [this](bool held)
            {
                scope.SetRunning(!held);
                runHold->setText(held ? "Run" : "Hold");
            });
        Configure();
    }

    void ScopePanel::Configure()
    {
        const auto& theme = ui::theme::Current();
        scope.SetChannelCount(3);
        scope.SetChannelConfig(0, { "Cmd", theme.Series(0) });
        scope.SetChannelConfig(1, { "Meas", theme.Series(1) });
        scope.SetChannelConfig(2, { "Err", theme.Series(2) });
        scope.SetSamplePeriod(0.01f);
        scope.SetTimePerDivision(0.5f);
        scope.SetTriggerMode(ui::scope::TriggerMode::Auto);
        scope.SetRunning(true);
    }

    void ScopePanel::Add(std::span<const arm_host::TraceSample> samples)
    {
        for (const auto& sample : samples)
        {
            const auto commanded = sample.commanded[joint].Value();
            const auto measured = sample.measured[joint].Value();
            const std::array values{ commanded, measured, commanded - measured };
            scope.AddSample(values);
        }
    }

    void ScopePanel::SelectJoint(std::size_t newJoint)
    {
        joints->setCurrentIndex(static_cast<int>(newJoint));
    }

    void ScopePanel::TriggerOnTrip()
    {
        scope.ForceTrigger();
    }

    void ScopePanel::Repaint()
    {
        plot->update();
    }

    EventLogPanel::EventLogPanel(const EventLog& log, QWidget* parent)
        : QWidget{ parent }
        , log{ log }
        , level{ new QComboBox{ this } }
        , joint{ new QComboBox{ this } }
        , text{ new QPlainTextEdit{ this } }
    {
        level->addItems({ "All levels", "Warnings and faults", "Faults" });
        level->setAccessibleName("Log level filter");
        joint->addItem("All joints");
        for (std::size_t index = 1; index <= arm::jointCount; ++index)
            joint->addItem(QString{ "Joint %1" }.arg(index));
        joint->setAccessibleName("Log joint filter");

        text->setReadOnly(true);
        text->setMaximumBlockCount(static_cast<int>(EventLog::defaultCapacity));
        text->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        text->setAccessibleName("Event log");

        auto* filters = new QHBoxLayout;
        filters->addWidget(level);
        filters->addWidget(joint);
        filters->addStretch();

        auto* layout = new QVBoxLayout{ this };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->addLayout(filters);
        layout->addWidget(text, 1);

        connect(level, &QComboBox::currentIndexChanged, this, &EventLogPanel::Rebuild);
        connect(joint, &QComboBox::currentIndexChanged, this, &EventLogPanel::Rebuild);
    }

    LogFilter EventLogPanel::Filter() const
    {
        LogFilter filter{ static_cast<LogLevel>(level->currentIndex()) };
        if (joint->currentIndex() > 0)
            filter.joint = static_cast<std::size_t>(joint->currentIndex() - 1);
        return filter;
    }

    void EventLogPanel::AppendNewest(std::size_t count)
    {
        const auto filter = Filter();
        const auto& lines = log.Lines();
        for (auto index = lines.size() - std::min(count, lines.size()); index < lines.size(); ++index)
            if (log.Passes(lines[index], filter))
                text->appendPlainText(QString::fromStdString(log.Format(lines[index])));
    }

    void EventLogPanel::Rebuild()
    {
        text->clear();
        AppendNewest(log.Lines().size());
    }

    bool EventLogPanel::SaveTo(const QString& path) const
    {
        QFile file{ path };
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            return false;

        QTextStream{ &file } << QString::fromStdString(log.Text({}));
        return true;
    }
}
