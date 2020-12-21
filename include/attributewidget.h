#ifndef ATTRIBUTEWIDGET_H
#define ATTRIBUTEWIDGET_H

#include "drawableattribute.h"

#include <QTreeWidget>
#include <QWidget>

namespace Ui {
class AttributeWidget;
}

class AttributeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AttributeWidget(DrawableAttribute*, QTreeWidget* a_pParent,
                             QTreeWidgetItem* a_pItem);
    ~AttributeWidget();

    DrawableAttribute *getAttribute() const;
    void setAttribute(DrawableAttribute *value);

private slots:
    void ExpansionRequired();

private:
    Ui::AttributeWidget *ui;
    DrawableAttribute* attribute;
    QTreeWidgetItem* m_pItem;
};

#endif // ATTRIBUTEWIDGET_H
