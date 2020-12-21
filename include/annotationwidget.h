#ifndef ANNOTATIONWIDGET_H
#define ANNOTATIONWIDGET_H

#include "drawableannotation.h"

#include <QTreeWidget>
#include <QWidget>

namespace Ui {
class AnnotationWidget;
}

class AnnotationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AnnotationWidget(DrawableAnnotation*, QTreeWidget* a_pParent,
                              QTreeWidgetItem* a_pItem);
    ~AnnotationWidget();

private slots:
    void ExpansionRequired();

private:
    Ui::AnnotationWidget *ui;
    DrawableAnnotation* annotation;
    QTreeWidgetItem* m_pItem;
};

#endif // ANNOTATIONWIDGET_H
