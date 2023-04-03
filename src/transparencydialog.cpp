#include "transparencydialog.h"
#include "ui_transparencydialog.h"

TransparencyDialog::TransparencyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TransparencyDialog)
{
    ui->setupUi(this);
    mesh = nullptr;
    ui->opacitySlider->setValue(static_cast<int>(0.6 * static_cast<double>(ui->opacitySlider->maximum())));
}

TransparencyDialog::~TransparencyDialog()
{
    delete ui;
}

void TransparencyDialog::on_buttonBox_accepted()
{
    this->mesh->setAnnotationsOpacity(static_cast<double>(this->ui->opacitySlider->value()) / 100);
    this->close();
}

DrawableMesh *TransparencyDialog::getMesh() const
{
    return mesh;
}

void TransparencyDialog::setMesh(DrawableMesh *value)
{
    mesh = value;
    originalOpacity = mesh->getAnnotationsOpacity();
    ui->opacitySlider->setValue(static_cast<int>(originalOpacity * static_cast<double>(ui->opacitySlider->maximum())));
}

void TransparencyDialog::on_opacitySlider_valueChanged(int value)
{
    if(mesh != nullptr)
    {
        this->mesh->setAnnotationsOpacity(static_cast<double>(value) / 100);
        emit(this->updateView(mesh));
    }
}

void TransparencyDialog::on_buttonBox_rejected()
{
    this->mesh->setAnnotationsOpacity(originalOpacity);
    emit(updateView(mesh));
    this->close();
}
