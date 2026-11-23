#pragma once

#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ArmControllersPresenter.hpp"
#include "tools/simulator/view_models/EventLog.hpp"
#include "tools/simulator/view_models/JogPresenter.hpp"
#include "tools/simulator/view_models/ProfileView.hpp"
#include "tools/simulator/view_models/TeachPresenter.hpp"
#include "tools/simulator/view_models/ToolReadout.hpp"
#include "tools/simulator/view_models/TuningPresenter.hpp"
#include "tools/simulator/view_models/VirtualMotorRows.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include "ui/scope/ScopeCore.hpp"
#include "ui/theme/Theme.hpp"
#include <QWidget>
#include <array>
#include <span>

class QComboBox;
class QKeyEvent;
class QVBoxLayout;
class QLineEdit;
class QRadioButton;
class QSlider;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;

namespace simulator
{
    class FormPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit FormPanel(QWidget* parent);

        ui::backend::qt::QtFormView& Form();
        void SetHint(const std::string& hint);

    private:
        ui::backend::qt::QtFormView* form;
        QLabel* hint;
    };

    class ConfigurationPanel
        : public FormPanel
    {
        Q_OBJECT

    public:
        explicit ConfigurationPanel(QWidget* parent);

        void SetState(const std::string& text, ui::theme::StatusLevel level);
        void ApplyTheme();

    private:
        QLabel* state;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
    };

    class ArmControllersPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit ArmControllersPanel(QWidget* parent);

        void Update(const ArmControllersPresenter& presenter);
        void ApplyTheme();

    signals:
        void chosen(int row);
        void actionTriggered(int action);

    private:
        QTableWidget* table;
        std::array<QPushButton*, armControllerActionCount> buttons{};
        QLabel* hint;
        std::vector<ArmControllerRow> shown;
    };

    class JogPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit JogPanel(QWidget* parent);

        void Update(const JogPresenter& presenter);
        void ApplyTheme();

    signals:
        void spaceChosen(int space);
        void shareChanged(int percent);
        void axisChosen(int axis);
        void pressed(int axis, int direction);
        void released();

    protected:
        bool eventFilter(QObject* watched, QEvent* event) override;

    private:
        void BuildSpace(QVBoxLayout& layout);
        void BuildRows(QVBoxLayout& layout);
        bool Key(const QKeyEvent& key, bool down);

        std::array<QRadioButton*, 3> spaces{};
        QSlider* share;
        QLabel* shareText;
        std::array<QLabel*, jogAxisCount> labels{};
        std::array<std::array<QPushButton*, 2>, jogAxisCount> buttons{};
        std::array<QLabel*, jogAxisCount> speeds{};
        QLabel* hint;
        QLabel* status;
        int keyAxis{ -1 };
    };

    class TeachPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit TeachPanel(QWidget* parent);

        void Update(const TeachPresenter& presenter);
        void ApplyTheme();

    signals:
        void chosen(int row);
        void nameEdited(const QString& name);
        void teach();
        void goTo();

    private:
        QTableWidget* table;
        QLineEdit* name;
        QPushButton* teachButton;
        QPushButton* goToButton;
        QLabel* hint;
        std::vector<TeachRow> shown;
    };

    class DisturbancesPanel
        : public FormPanel
    {
        Q_OBJECT

    public:
        explicit DisturbancesPanel(QWidget* parent);

        void SetActive(const std::string& text);
        void ApplyTheme();

    private:
        QLabel* mark;
        QLabel* active;
    };

    class TuningPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit TuningPanel(QWidget* parent);

        ui::backend::qt::QtFormView& Form();
        QPushButton* ExportButton() const;
        void Update(const TuningPresenter& presenter);
        void ApplyTheme();

    private:
        void BuildMetrics();

        ui::charts::LinearAxis axis{ "Time", 6, 2, "t = ", "s" };
        ui::charts::ChartCore chart{ axis, {} };
        FormPanel* controls;
        QLabel* status;
        QPushButton* exportCsv;
        ui::backend::qt::QtPaintedWidget* plot;
        QTableWidget* metrics;
        uint32_t chartRevision{ 0 };
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
    };

    class ToolPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit ToolPanel(QWidget* parent);

        void Update(const ToolReadout& readout);
        void ApplyTheme();

    private:
        QLabel* position;
        QLabel* rotation;
        QLabel* manipulability;
        QLabel* singularity;
        ToolReadout shown;
    };

    class ProfilePanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit ProfilePanel(QWidget* parent);

        void Update(const ProfileView& view, const QString& path);

    private:
        QLabel* name;
        QLabel* file;
        QLabel* summary;
        QTableWidget* table;
        ProfileView shown;
        QString shownPath;
    };

    class VirtualMotorsPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit VirtualMotorsPanel(QWidget* parent);

        void Update(const VirtualMotorsView& view);
        void ApplyTheme();

    signals:
        void restartRequested(int joint);

    private:
        void Render();

        QLabel* mark;
        QTableWidget* table;
        std::array<QLabel*, arm::jointCount> processes{};
        std::array<QPushButton*, arm::jointCount> restarts{};
        QLabel* folder;
        QLabel* hint;
        VirtualMotorsView shown;
    };

    class ScopePanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit ScopePanel(QWidget* parent);

        void Add(std::span<const arm_host::TraceSample> samples);
        void SelectJoint(std::size_t joint);
        void TriggerOnTrip();
        void Repaint();

    private:
        void Configure();

        void ShowLost();

        ui::scope::ScopeCore scope;
        QComboBox* joints;
        QPushButton* runHold;
        QLabel* lost;
        ui::backend::qt::QtPaintedWidget* plot;
        std::size_t joint{ 0 };
        uint64_t lostSamples{ 0 };
    };

    class EventLogPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        EventLogPanel(const EventLog& log, QWidget* parent);

        void AppendNewest(std::size_t count);
        void Rebuild();
        bool SaveTo(const QString& path) const;

    private:
        LogFilter Filter() const;

        const EventLog& log;
        QComboBox* level;
        QComboBox* joint;
        QPlainTextEdit* text;
    };
}
