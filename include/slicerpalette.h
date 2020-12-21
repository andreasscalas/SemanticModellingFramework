#ifndef SLICERPALETTE_H
#define SLICERPALETTE_H

#include <QWidget>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <qslider.h>
#include <QVTKWidget.h>
#include <qtabwidget.h>
#include <qcheckbox.h>

namespace Ui {
class SlicerPalette;
}

class SlicerPalette : public QWidget
{
    Q_OBJECT

public:
    explicit SlicerPalette(QWidget *parent = 0);
    ~SlicerPalette();

    QVTKWidget* getSlicerWidget();
    QTabWidget* getTabWidget();
    QSlider* getLevelOfDetailSlider();
    QSlider* getMaxErrorSlider();
    QSlider* getXSlider();
    QSlider* getYSlider();
    QSlider* getZSlider();
    QSlider* getXSlider2();
    QSlider* getYSlider2();
    QSlider* getZSlider2();
    QCheckBox* getClusterCheckBox();

private:
    Ui::SlicerPalette *ui;

};

#endif // SLICERPALETTE_H
