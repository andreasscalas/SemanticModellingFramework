#ifndef CATEGORYBUTTON_H
#define CATEGORYBUTTON_H

#include <QPushButton>
#include <QTreeWidget>

class CategoryButton : public QPushButton
{
    Q_OBJECT
    public:
        CategoryButton(const QString& a_Text, QTreeWidget* a_pParent,
            QTreeWidgetItem* a_pItem);

    private slots:
        void ButtonPressed();

    private:
        QTreeWidgetItem* m_pItem;
};

#endif // CATEGORYBUTTON_H
