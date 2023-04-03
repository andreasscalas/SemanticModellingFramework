#include "checkablelist.h"

CheckableList::CheckableList(QWidget *parent) : QScrollArea(parent){
    containerLayout = new QVBoxLayout();
    containerLayout->setAlignment(Qt::AlignTop);
}

QCheckBox* CheckableList::addItem(std::string str)
{
    return this->addItem(QString::fromStdString(str));
}

QCheckBox* CheckableList::addItem(QString str)
{
    QCheckBox *checkbox = new QCheckBox(str);
    checkboxes.push_back(checkbox);
    containerLayout->addWidget(checkbox);
    containerLayout->update();
    this->widget()->setLayout(containerLayout);
    this->widget()->update();
    this->update();
    return checkbox;
}

void CheckableList::removeItem(QCheckBox * checkbox)
{
    checkbox->setVisible(false);
    containerLayout->removeWidget(checkbox);
    containerLayout->update();
    checkboxes.erase(std::find(checkboxes.begin(), checkboxes.end(), checkbox));
    delete checkbox;
    this->widget()->update();
    this->update();
}
