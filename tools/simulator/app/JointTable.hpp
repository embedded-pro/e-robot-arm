#ifndef TOOLS_SIMULATOR_APP_JOINT_TABLE_HPP
#define TOOLS_SIMULATOR_APP_JOINT_TABLE_HPP

#include "tools/simulator/view_models/JointRows.hpp"
#include <QTableWidget>
#include <optional>

namespace simulator
{
    class JointTable
        : public QTableWidget
    {
        Q_OBJECT

    public:
        explicit JointTable(QWidget* parent);

        void Update(const std::array<JointRow, arm::jointCount>& rows);
        void Select(std::optional<std::size_t> joint);

    signals:
        void jointSelected(int joint);

    private:
        void SetCell(int row, int column, const std::string& text, Qt::Alignment alignment);
    };
}

#endif
