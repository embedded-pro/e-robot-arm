#pragma once

#include "tools/simulator/view_models/AdapterChoices.hpp"
#include <QDialog>
#include <QString>
#include <cstdint>

class QComboBox;
class QLabel;
class QPushButton;

namespace simulator
{
    class AdapterDialog
        : public QDialog
    {
        Q_OBJECT

    public:
        AdapterDialog(const QString& lastUsed, QWidget* parent);

        uint32_t StartSearch();
        void Show(uint32_t search, const AdapterChoices& choices);
        void TimeOut(uint32_t search);
        QString Adapter() const;
        AdapterSearch State() const;

    signals:
        void searchRequested();

    private:
        QString lastUsed;
        QLabel* label;
        QComboBox* adapters;
        QPushButton* searchAgain;
        uint32_t search{ 0 };
        AdapterSearch state{ AdapterSearch::searching };
    };
}
