#include "tools/simulator/app/JointTable.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QFontDatabase>
#include <QHeaderView>
#include <QProgressBar>

namespace simulator
{
    namespace
    {
        enum Column
        {
            numberColumn,
            positionColumn,
            speedColumn,
            gravityColumn,
            stateColumn,
            faultColumn,
            limitColumn,
            columns
        };

        ui::theme::ColorRole RoleFor(ui::theme::StatusLevel level)
        {
            switch (level)
            {
                case ui::theme::StatusLevel::Ok:
                    return ui::theme::ColorRole::Ok;
                case ui::theme::StatusLevel::Warning:
                    return ui::theme::ColorRole::Warning;
                case ui::theme::StatusLevel::Fault:
                    return ui::theme::ColorRole::Fault;
                default:
                    return ui::theme::ColorRole::Text;
            }
        }
    }

    JointTable::JointTable(QWidget* parent)
        : QTableWidget{ static_cast<int>(arm::jointCount), columns, parent }
    {
        setObjectName("jointTable");
        setAccessibleName("Joints");
        setHorizontalHeaderLabels({ "#", "Position", "Speed", "Gravity", "State", "Fault", "Limit" });
        verticalHeader()->setVisible(false);
        horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        horizontalHeader()->setSectionResizeMode(limitColumn, QHeaderView::Stretch);
        setSelectionBehavior(QAbstractItemView::SelectRows);
        setSelectionMode(QAbstractItemView::SingleSelection);
        setEditTriggers(QAbstractItemView::NoEditTriggers);
        setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setMinimumHeight(horizontalHeader()->sizeHint().height() + rowCount() * verticalHeader()->defaultSectionSize() + 2 * frameWidth());

        for (int row = 0; row != rowCount(); ++row)
        {
            auto* bar = new QProgressBar{ this };
            bar->setRange(0, 1000);
            bar->setTextVisible(false);
            bar->setMaximumHeight(8);
            bar->setAccessibleName(QString{ "Joint %1 position within its limits" }.arg(row + 1));
            setCellWidget(row, limitColumn, bar);
        }

        connect(this, &QTableWidget::itemSelectionChanged, this, [this]()
            {
                const auto selected = selectionModel()->selectedRows();
                if (!selected.isEmpty())
                    emit jointSelected(selected.front().row());
            });
    }

    void JointTable::Update(const std::array<JointRow, arm::jointCount>& rows)
    {
        for (int row = 0; row != rowCount(); ++row)
        {
            const auto& joint = rows[static_cast<std::size_t>(row)];
            SetCell(row, numberColumn, joint.number, Qt::AlignCenter);
            SetCell(row, positionColumn, joint.position, Qt::AlignRight | Qt::AlignVCenter);
            SetCell(row, speedColumn, joint.speed, Qt::AlignRight | Qt::AlignVCenter);
            SetCell(row, gravityColumn, joint.gravity, Qt::AlignRight | Qt::AlignVCenter);
            SetCell(row, stateColumn, joint.state, Qt::AlignLeft | Qt::AlignVCenter);
            SetCell(row, faultColumn, joint.fault, Qt::AlignLeft | Qt::AlignVCenter);

            const auto stateColor = ui::backend::qt::ToQt(ui::theme::Current().Get(RoleFor(joint.level)));
            item(row, stateColumn)->setForeground(stateColor);
            item(row, faultColumn)->setForeground(stateColor);
            static_cast<QProgressBar*>(cellWidget(row, limitColumn))->setValue(static_cast<int>(joint.limitFraction * 1000.0f));
        }
    }

    void JointTable::Select(std::optional<std::size_t> joint)
    {
        const QSignalBlocker blocker{ this };
        if (joint)
            selectRow(static_cast<int>(*joint));
        else
            clearSelection();
    }

    void JointTable::SetCell(int row, int column, const std::string& text, Qt::Alignment alignment)
    {
        auto* cell = item(row, column);
        if (cell == nullptr)
        {
            cell = new QTableWidgetItem;
            cell->setTextAlignment(alignment);
            setItem(row, column, cell);
        }
        const auto value = QString::fromStdString(text);
        if (cell->text() != value)
            cell->setText(value);
    }
}
