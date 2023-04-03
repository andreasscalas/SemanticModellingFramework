#ifndef MEASURESLISTWIDGET_H
#define MEASURESLISTWIDGET_H

#include "drawablemesh.h"

#include <QTreeWidget>

namespace Ui {
class MeasuresListWidget;
}

class MeasuresListWidget : public QTreeWidget
{
    Q_OBJECT

public:
    explicit MeasuresListWidget(QWidget *parent = nullptr);
    ~MeasuresListWidget();

    void update();
    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

signals:
    void updateSignal();
    void updateViewSignal();

private slots:
    void updateSlot();
    void updateViewSlot();
private:
    Ui::MeasuresListWidget *ui;
    DrawableMesh* mesh;
};

#endif // MEASURESLISTWIDGET_H
