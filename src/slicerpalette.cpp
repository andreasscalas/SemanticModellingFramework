#include "slicerpalette.h"
#include "ui_slicerpalette.h"

SlicerPalette::SlicerPalette(QWidget *parent) : QWidget(parent), ui(new Ui::SlicerPalette){
    ui->setupUi(this);
}

SlicerPalette::~SlicerPalette()
{
    delete ui;
}

QVTKOpenGLNativeWidget *SlicerPalette::getSlicerWidget()
{
    return this->ui->slicerWidget;
}

QTabWidget *SlicerPalette::getTabWidget()
{
    return this->ui->sliceTabWidget;
}

QSlider *SlicerPalette::getLevelOfDetailSlider()
{
    return this->ui->levelOfDetailSlider;
}

QSlider* SlicerPalette::getMaxErrorSlider()
{
    return this->ui->maxErrorSlider;
}

QSlider* SlicerPalette::getXSlider()
{
    return this->ui->xSlider;
}

QSlider* SlicerPalette::getYSlider()
{
    return this->ui->ySlider;
}

QSlider* SlicerPalette::getZSlider()
{
    return this->ui->zSlider;
}

QSlider* SlicerPalette::getXSlider2()
{
    return this->ui->xSlider2;
}

QSlider* SlicerPalette::getYSlider2()
{
    return this->ui->ySlider2;
}

QSlider* SlicerPalette::getZSlider2()
{
    return this->ui->zSlider2;
}

QCheckBox *SlicerPalette::getClusterCheckBox()
{
    return this->ui->clusterCheckBox;
}

QCheckBox *SlicerPalette::getShowSlicesCheckBox()
{
    return this->ui->showSlices;
}

QCheckBox *SlicerPalette::getShowSkeletonCheckBox()
{
    return this->ui->showSkeleton;
}

QCheckBox *SlicerPalette::getShowNodesCheckBox()
{
    return this->ui->showNodesBox;
}

QCheckBox *SlicerPalette::getShowArcNodesCheckBox()
{
    return this->ui->showArcNodesBox;
}

QCheckBox *SlicerPalette::getShowBoundingBoxCheckBox()
{
    return this->ui->showBoundingBox;
}

QCheckBox *SlicerPalette::getShowBoundingRectangleCheckBox()
{
    return this->ui->showBoundingRectangle;
}

QCheckBox *SlicerPalette::getShowFittedCircleCheckBox()
{
    return this->ui->showFittedCircle;
}

QCheckBox *SlicerPalette::getShowDiagonalsCheckBox()
{
    return this->ui->showDiagonals;
}

QCheckBox *SlicerPalette::getShowCentersCheckBox()
{
    return this->ui->showCenters;
}

QCheckBox *SlicerPalette::getShowConvexHull()
{
    return this->ui->showConvexHull;
}

QSpinBox *SlicerPalette::getClusterNumberBox()
{
    return this->ui->clusterNumberBox;
}
