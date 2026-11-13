#ifndef TOOLS_SIMULATOR_APP_PANELS_HPP
#define TOOLS_SIMULATOR_APP_PANELS_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/EventLog.hpp"
#include "tools/simulator/view_models/ProfileView.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/scope/ScopeCore.hpp"
#include "ui/theme/Theme.hpp"
#include <QWidget>
#include <span>

class QComboBox;
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

    class MotorControllerPanel
        : public FormPanel
    {
        Q_OBJECT

    public:
        explicit MotorControllerPanel(QWidget* parent);

        void SetState(const std::string& text, ui::theme::StatusLevel level);
        void ApplyTheme();

    private:
        QLabel* state;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
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

        ui::scope::ScopeCore scope;
        QComboBox* joints;
        QPushButton* runHold;
        ui::backend::qt::QtPaintedWidget* plot;
        std::size_t joint{ 0 };
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

#endif
