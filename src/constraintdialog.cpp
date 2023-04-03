#include "constraintdialog.h"
#include "ui_constraintdialog.h"

ConstraintDialog::ConstraintDialog(QWidget *parent) : QDialog(parent), ui(new Ui::ConstraintDialog){
    ui->setupUi(this);
    connect(this->ui->typeList, SIGNAL(currentRowChanged(int)), this, SLOT(slotType(int)));
	connect(this->ui->weightSpinBox, SIGNAL(valueChanged(int)), this, SLOT(slotWeight(int)));
    connect(this->ui->RelativeOption, SIGNAL(currentRowChanged(int)), this, SLOT(slotRelative(int)));
    connect(this->ui->SaveButton, SIGNAL(clicked()), this, SLOT(slotSave()));
    connect(this->ui->CancelButton, SIGNAL(clicked()), this, SLOT(slotCancel()));
}

ConstraintDialog::~ConstraintDialog(){
    delete ui;
}

void ConstraintDialog::slotType(int type)
{
    switch (type)
    {
    case 0:
        this->type = Utilities::ConstraintType::EDGE_STRAIN;
        break;
    case 1:
        this->type = Utilities::ConstraintType::TRIANGLE_STRAIN;
        break;
    case 2:
        this->type = Utilities::ConstraintType::TETRAHEDRON_STRAIN;
        break;
    case 3:
        this->type = Utilities::ConstraintType::AREA;
        break;
    case 4:
        this->type = Utilities::ConstraintType::VOLUME;
        break;
    case 5:
        this->type = Utilities::ConstraintType::BENDING;
        break;
    case 6:
        this->type = Utilities::ConstraintType::CLOSENESS;
        break;
    case 7:
        this->type = Utilities::ConstraintType::LINE;
        break;
    case 8:
        this->type = Utilities::ConstraintType::PLANE;
        break;
    case 9:
        this->type = Utilities::ConstraintType::CIRCLE;
        break;
    case 10:
        this->type = Utilities::ConstraintType::SPHERE;
        break;
    case 11:
        this->type = Utilities::ConstraintType::SIMILARITY;
        break;
    case 12:
        this->type = Utilities::ConstraintType::RIGID;
        break;
    case 13:
        this->type = Utilities::ConstraintType::RECTANGLE;
        break;
    case 14:
        this->type = Utilities::ConstraintType::PARALLELOGRAM;
        break;
    case 15:
        this->type = Utilities::ConstraintType::LAPLACIAN;
        break;
    case 16:
        this->type = Utilities::ConstraintType::LAPLACIAN_DISPLACEMENT;
        break;
    default:
        assert(false);
    }

}

void ConstraintDialog::slotWeight(int weight)
{

    this->weight = weight;
}

void ConstraintDialog::slotRelative(int relative)
{

    this->relative = relative == 0;

}

void ConstraintDialog::slotCancel()
{
    this->close();
}


void ConstraintDialog::slotSave(){
    this->close();
    emit addConstraint(type, weight);
}
