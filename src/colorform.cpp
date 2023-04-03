#include "colorform.h"
#include <QColorDialog>

ColorForm::ColorForm(QWidget* parent) : QDialog(parent){setupUi(this);}

void ColorForm::on_meshButton_clicked(){

    meshColor = QColorDialog::getColor();
    emit(meshColorChanged(meshColor));

}

void ColorForm::on_cageButton_clicked(){

    cageColor = QColorDialog::getColor();
    emit(cageColorChanged(cageColor));

}
