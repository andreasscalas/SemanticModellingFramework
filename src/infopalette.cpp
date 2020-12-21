#include "infopalette.h"
#include "ui_infopalette.h"

InfoPalette::InfoPalette(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::InfoPalette)
{
    ui->setupUi(this);
}

InfoPalette::~InfoPalette()
{
    delete ui;
}

void InfoPalette::updateInfo()
{
//    this->ui->constraintSelectionBox->clear();
//    for(unsigned int i = 0; i < constraints.size(); i++){
//        this->ui->constraintSelectionBox->addItem(QString::fromStdString(constraints[i]->getInfo().type));
//    }

}

std::vector<std::shared_ptr<ShapeOp::Constraint> > InfoPalette::getConstraints() const
{
    return constraints;
}

void InfoPalette::setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value)
{
    constraints = value;
}
