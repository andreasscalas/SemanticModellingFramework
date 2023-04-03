#ifndef SLICERPALETTE_H
#define SLICERPALETTE_H

#include <QVTKOpenGLNativeWidget.h>

#include <QWidget>
#include <QSlider>
#include <QTabWidget>
#include <QCheckBox>
#include <QSpinBox>

namespace Ui {
class SlicerPalette;
}

class SlicerPalette : public QWidget
{
    Q_OBJECT

public:
    explicit SlicerPalette(QWidget *parent = 0);
    ~SlicerPalette();

    QVTKOpenGLNativeWidget* getSlicerWidget();
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
    QCheckBox* getShowSlicesCheckBox();
    QCheckBox* getShowSkeletonCheckBox();
    QCheckBox* getShowNodesCheckBox();
    QCheckBox* getShowArcNodesCheckBox();
    QCheckBox* getShowBoundingBoxCheckBox();
    QCheckBox* getShowBoundingRectangleCheckBox();
    QCheckBox* getShowFittedCircleCheckBox();
    QCheckBox* getShowDiagonalsCheckBox();
    QCheckBox* getShowCentersCheckBox();
    QCheckBox* getShowConvexHull();
    QSpinBox* getClusterNumberBox();


private:
    Ui::SlicerPalette *ui;

};

#endif // SLICERPALETTE_H
