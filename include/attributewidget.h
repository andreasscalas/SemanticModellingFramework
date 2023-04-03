#ifndef ATTRIBUTEWIDGET_H
#define ATTRIBUTEWIDGET_H

#include "drawableattribute.h"

#include <QPushButton>
#include <QTreeWidget>

namespace Ui {
class AttributeWidget;
}

class AttributeWidget : public QTreeWidget
{
    Q_OBJECT

public:
    explicit AttributeWidget(QWidget* parent);
    ~AttributeWidget();

    void update();

    Annotation *getAnnotation() const;
    void setAnnotation(Annotation *value);

signals:
    void updateSignal();
    void updateViewSignal();
private slots:
    void measureButtonClickedSlot();
    void deletionButtonClickedSlot();

private:
    std::map<QPushButton*, Attribute*>  buttonMeasureMap;
    Ui::AttributeWidget *ui;
    Annotation* annotation;
    QTreeWidgetItem* m_pItem;
};

#endif // ATTRIBUTEWIDGET_H
