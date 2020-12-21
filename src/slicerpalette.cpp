#include "slicerpalette.h"
#include "ui_slicerpalette.h"

SlicerPalette::SlicerPalette(QWidget *parent) : QWidget(parent), ui(new Ui::SlicerPalette){
    ui->setupUi(this);
}

SlicerPalette::~SlicerPalette()
{
    delete ui;
}

QVTKWidget *SlicerPalette::getSlicerWidget()
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
