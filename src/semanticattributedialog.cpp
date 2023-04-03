#include "semanticattributedialog.h"
#include "ui_semanticattributedialog.h"

SemanticAttributeDialog::SemanticAttributeDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SemanticAttributeDialog)
{
    ui->setupUi(this);
    success = false;
}

SemanticAttributeDialog::~SemanticAttributeDialog()
{
    delete ui;
}

void SemanticAttributeDialog::on_buttonBox_accepted()
{
    success = true;
}

bool SemanticAttributeDialog::getSuccess() const
{
    return success;
}

QString SemanticAttributeDialog::getAttributeName()
{
    return this->ui->nameTextEdit->toPlainText();
}

QString SemanticAttributeDialog::getAttributeValue()
{

    return this->ui->valueTextEdit->toPlainText();
}
