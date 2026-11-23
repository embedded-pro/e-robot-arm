#include "tools/simulator/app/AdapterDialog.hpp"
#include <QComboBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace simulator
{
    AdapterDialog::AdapterDialog(const QString& lastUsed, QWidget* parent)
        : QDialog(parent)
        , lastUsed(lastUsed)
        , label(new QLabel{ this })
        , adapters(new QComboBox{ this })
        , searchAgain(new QPushButton{ "&Search Again", this })
    {
        setObjectName("adapterDialog");
        setWindowTitle("Real Arm");
        setAttribute(Qt::WA_DeleteOnClose);

        label->setObjectName("adapterSearchState");
        label->setWordWrap(true);
        adapters->setObjectName("adapterChoice");
        adapters->setEditable(true);
        adapters->setAccessibleName("CAN adapter");
        label->setBuddy(adapters);
        searchAgain->setObjectName("adapterSearchAgain");

        auto* buttons = new QDialogButtonBox{ QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this };
        buttons->button(QDialogButtonBox::Ok)->setText("Connect…");
        buttons->addButton(searchAgain, QDialogButtonBox::ActionRole);

        auto* layout = new QVBoxLayout{ this };
        layout->addWidget(label);
        layout->addWidget(adapters);
        layout->addWidget(buttons);

        connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(searchAgain, &QPushButton::clicked, this, &AdapterDialog::searchRequested);
    }

    uint32_t AdapterDialog::StartSearch()
    {
        Show(++search, SearchingForAdapters(adapters->currentText().isEmpty() ? lastUsed.toStdString() : adapters->currentText().toStdString()));
        return search;
    }

    void AdapterDialog::Show(uint32_t answering, const AdapterChoices& choices)
    {
        if (answering != search)
            return;

        state = choices.search;
        label->setText(QString::fromStdString(choices.label));
        adapters->clear();
        for (const auto& item : choices.items)
            adapters->addItem(QString::fromStdString(item));
        adapters->setCurrentIndex(choices.current);
        searchAgain->setEnabled(state != AdapterSearch::searching);
    }

    void AdapterDialog::TimeOut(uint32_t answering)
    {
        if (answering == search && state == AdapterSearch::searching)
            Show(answering, AdapterSearchTimedOut(adapters->currentText().toStdString()));
    }

    QString AdapterDialog::Adapter() const
    {
        return adapters->currentText().trimmed();
    }

    AdapterSearch AdapterDialog::State() const
    {
        return state;
    }
}
