#include "tools/simulator/app/Panels.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QComboBox>
#include <QFile>
#include <QFontDatabase>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
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

    DisturbancesPanel::DisturbancesPanel(QWidget* parent)
        : FormPanel{ parent }
        , mark{ new QLabel{ "Simulation only", this } }
        , active{ new QLabel{ this } }
    {
        mark->setAccessibleName("Simulation only: these disturbances reach simulated joints and virtual motors alone");
        mark->setAlignment(Qt::AlignRight);
        active->setWordWrap(true);
        active->setAccessibleName("Disturbances applied");
        auto* panelLayout = static_cast<QVBoxLayout*>(layout());
        panelLayout->insertWidget(0, mark);
        panelLayout->insertWidget(2, active);
        ApplyTheme();
    }

    void DisturbancesPanel::SetActive(const std::string& text)
    {
        const auto value = QString::fromStdString(text);
        if (active->text() != value)
            active->setText(value);
    }

    void DisturbancesPanel::ApplyTheme()
    {
        ui::backend::qt::StyleStatusLabel(*mark, ui::theme::StatusLevel::Warning);
    }

    TuningPanel::TuningPanel(QWidget* parent)
        : QWidget{ parent }
        , controls{ new FormPanel{ this } }
        , status{ new QLabel{ this } }
        , exportCsv{ new QPushButton{ "Export CSV…", this } }
        , plot{ new ui::backend::qt::QtPaintedWidget{ chart, this } }
        , metrics{ new QTableWidget{ static_cast<int>(tuningMetricCount), 3, this } }
    {
        status->setAccessibleName("Tuning run state");
        status->setWordWrap(true);
        exportCsv->setAccessibleName("Export the last run as CSV");
        exportCsv->setEnabled(false);
        plot->SetBackgroundRole(ui::theme::ColorRole::ScopeBackground);
        plot->setAccessibleName("Tuning run plot");
        plot->setMinimumSize(320, 200);
        BuildMetrics();

        auto* column = new QWidget{ this };
        auto* columnLayout = new QVBoxLayout{ column };
        columnLayout->setContentsMargins(0, 0, 0, 0);
        columnLayout->addWidget(controls, 1);
        auto* result = new QVBoxLayout;
        result->setContentsMargins(8, 0, 8, 8);
        result->addWidget(status);
        result->addWidget(exportCsv, 0, Qt::AlignLeft);
        columnLayout->addLayout(result);
        column->setMinimumWidth(260);

        auto* split = new QSplitter{ Qt::Horizontal, this };
        split->addWidget(column);
        split->addWidget(plot);
        split->addWidget(metrics);
        split->setStretchFactor(0, 0);
        split->setStretchFactor(1, 1);
        split->setStretchFactor(2, 0);
        split->setSizes({ 300, 900, 320 });

        auto* panelLayout = new QHBoxLayout{ this };
        panelLayout->setContentsMargins(0, 0, 0, 0);
        panelLayout->addWidget(split);
        ApplyTheme();
    }

    ui::backend::qt::QtFormView& TuningPanel::Form()
    {
        return controls->Form();
    }

    void TuningPanel::BuildMetrics()
    {
        metrics->setHorizontalHeaderLabels({ "Metric", "This run", "Reference" });
        metrics->verticalHeader()->setVisible(false);
        metrics->setEditTriggers(QAbstractItemView::NoEditTriggers);
        metrics->setSelectionMode(QAbstractItemView::NoSelection);
        metrics->setAccessibleName("Tuning metrics");
        metrics->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        for (int row = 0; row != static_cast<int>(tuningMetricCount); ++row)
            for (int column = 0; column != 3; ++column)
            {
                auto* cell = new QTableWidgetItem;
                if (column != 0)
                    cell->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
                metrics->setItem(row, column, cell);
            }
    }

    QPushButton* TuningPanel::ExportButton() const
    {
        return exportCsv;
    }

    void TuningPanel::Update(const TuningPresenter& presenter)
    {
        controls->SetHint(presenter.Hint());
        const auto text = QString::fromStdString(presenter.Status());
        if (status->text() != text)
            status->setText(text);
        if (presenter.StatusLevel() != level)
        {
            level = presenter.StatusLevel();
            ui::backend::qt::StyleStatusLabel(*status, level);
        }
        exportCsv->setEnabled(presenter.CanExport());

        for (std::size_t row = 0; row != tuningMetricCount; ++row)
        {
            const auto& metric = presenter.Metrics()[row];
            const std::array values{ QString::fromUtf8(metric.name.data(), static_cast<int>(metric.name.size())), QString::fromStdString(metric.current),
                QString::fromStdString(metric.reference) };
            for (int column = 0; column != 3; ++column)
                if (auto* cell = metrics->item(static_cast<int>(row), column); cell->text() != values[column])
                    cell->setText(values[column]);
        }

        if (presenter.ChartRevision() == chartRevision)
            return;
        chartRevision = presenter.ChartRevision();
        chart.SetAxisValues(presenter.Chart().time);
        chart.SetPanels(presenter.Chart().panels);
        plot->update();
    }

    void TuningPanel::ApplyTheme()
    {
        ui::backend::qt::StyleStatusLabel(*status, level);
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

    ToolPanel::ToolPanel(QWidget* parent)
        : QWidget{ parent }
        , position{ new QLabel{ "—", this } }
        , rotation{ new QLabel{ "—", this } }
        , manipulability{ new QLabel{ "—", this } }
        , singularity{ new QLabel{ "—", this } }
    {
        const auto fixed = QFontDatabase::systemFont(QFontDatabase::FixedFont);
        auto* layout = new QGridLayout{ this };
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setHorizontalSpacing(16);
        layout->setVerticalSpacing(4);
        const std::array<std::pair<const char*, QLabel*>, 4> rows{ { { "Position", position }, { "Rotation", rotation }, { "Manipulability", manipulability },
            { "Singularity", singularity } } };
        for (int row = 0; row != static_cast<int>(rows.size()); ++row)
        {
            auto* name = new QLabel{ rows[static_cast<std::size_t>(row)].first, this };
            auto* value = rows[static_cast<std::size_t>(row)].second;
            name->setBuddy(value);
            value->setFont(fixed);
            value->setAccessibleName(QString{ "Tool " } + rows[static_cast<std::size_t>(row)].first);
            value->setTextInteractionFlags(Qt::TextSelectableByMouse);
            layout->addWidget(name, row, 0);
            layout->addWidget(value, row, 1);
        }
        layout->setColumnStretch(1, 1);
        ui::backend::qt::StyleStatusLabel(*singularity, ui::theme::StatusLevel::Neutral);
    }

    void ToolPanel::Update(const ToolReadout& readout)
    {
        if (readout == shown)
            return;

        const bool levelChanged = readout.singularityLevel != shown.singularityLevel;
        shown = readout;
        position->setText(QString::fromStdString(readout.position));
        rotation->setText(QString::fromStdString(readout.rotation));
        manipulability->setText(QString::fromStdString(readout.manipulability));
        singularity->setText(QString::fromStdString(readout.singularity));
        if (levelChanged)
            ApplyTheme();
    }

    void ToolPanel::ApplyTheme()
    {
        ui::backend::qt::StyleStatusLabel(*singularity, shown.singularityLevel);
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

    VirtualMotorsPanel::VirtualMotorsPanel(QWidget* parent)
        : QWidget{ parent }
        , mark{ new QLabel{ "Simulation only", this } }
        , table{ new QTableWidget{ static_cast<int>(arm::jointCount), 4, this } }
        , folder{ new QLabel{ this } }
        , hint{ new QLabel{ this } }
    {
        mark->setAccessibleName("Simulation only: these motors are processes on this computer");
        folder->setAccessibleName("Folder of the motors' logs and memory");
        folder->setTextInteractionFlags(Qt::TextSelectableByMouse);
        folder->setWordWrap(true);
        hint->setAccessibleName("Why Restart is disabled");
        hint->setWordWrap(true);

        table->setObjectName("virtualMotorsTable");
        table->setAccessibleName("Virtual motor per joint");
        table->setHorizontalHeaderLabels({ "Joint", "Motor", "Process", "" });
        table->verticalHeader()->setVisible(false);
        table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setSelectionMode(QAbstractItemView::NoSelection);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto row = static_cast<int>(joint);
            processes[joint] = new QLabel{ table };
            restarts[joint] = new QPushButton{ "Restart", table };
            restarts[joint]->setAccessibleName(QString{ "Restart joint %1's motor" }.arg(row + 1));
            table->setCellWidget(row, 2, processes[joint]);
            table->setCellWidget(row, 3, restarts[joint]);
            connect(restarts[joint], &QPushButton::clicked, this, [this, row]()
                {
                    emit restartRequested(row);
                });
        }

        auto* header = new QHBoxLayout{};
        header->addStretch();
        header->addWidget(mark);
        auto* layout = new QVBoxLayout{ this };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->addLayout(header);
        layout->addWidget(table);
        layout->addWidget(folder);
        layout->addWidget(hint);
        Render();
        ApplyTheme();
    }

    void VirtualMotorsPanel::Update(const VirtualMotorsView& view)
    {
        if (view == shown)
            return;

        shown = view;
        Render();
    }

    void VirtualMotorsPanel::Render()
    {
        const auto& view = shown;
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& motor = view.rows[joint];
            const auto row = static_cast<int>(joint);
            table->setItem(row, 0, new QTableWidgetItem{ ui::backend::qt::ToQt(motor.joint) });
            table->setItem(row, 1, new QTableWidgetItem{ ui::backend::qt::ToQt(motor.motor) });
            processes[joint]->setText(ui::backend::qt::ToQt(motor.process));
            ui::backend::qt::StyleStatusLabel(*processes[joint], motor.level);
            restarts[joint]->setEnabled(motor.restartEnabled);
            restarts[joint]->setToolTip(motor.restartEnabled ? QString{ "Power-cycle this joint's virtual motor with the profile's motor" }
                                                             : ui::backend::qt::ToQt(view.hint));
        }
        folder->setText(ui::backend::qt::ToQt(view.folder));
        hint->setText(ui::backend::qt::ToQt(view.hint));
        hint->setVisible(!view.hint.empty());
    }

    void VirtualMotorsPanel::ApplyTheme()
    {
        ui::backend::qt::StyleStatusLabel(*mark, ui::theme::StatusLevel::Warning);
        ui::backend::qt::StyleStatusLabel(*hint, ui::theme::StatusLevel::Neutral);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            ui::backend::qt::StyleStatusLabel(*processes[joint], shown.rows[joint].level);
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
