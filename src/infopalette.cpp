#include "infopalette.h"
#include "ui_infopalette.h"

#include <QStringListModel>

InfoPalette::InfoPalette(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::InfoPalette)
{
    ui->setupUi(this);
    connect(this->ui->constraintSelectionBox, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChangedSlot(int)));
}

InfoPalette::~InfoPalette()
{
    delete ui;
}

void InfoPalette::updateInfo()
{
    this->ui->constraintSelectionBox->clear();
    for(unsigned int i = 0; i < constraints.size(); i++){
        this->ui->constraintSelectionBox->addItem(QString::fromStdString(constraints[i]->getType()) + " with id " + QString::number(constraints[i]->getId()));
    }

}

std::vector<AnnotationsConstraint *> InfoPalette::getConstraints() const
{
    return constraints;
}

void InfoPalette::setConstraints(const std::vector<AnnotationsConstraint *> &value)
{
    constraints = value;
}

DrawableMesh *InfoPalette::getMesh() const
{
    return mesh;
}

void InfoPalette::setMesh(DrawableMesh *value)
{
    mesh = value;
}

void InfoPalette::currentIndexChangedSlot(int pos)
{
    if(pos >= 0)
    {
        QStringList list;
        QStringListModel *model = new QStringListModel(this->ui->listView);
        std::stringstream ss;
        constraints[pos]->print(ss);
        std::string line;
        while(!ss.eof()){
            std::string line;
            getline(ss, line, '\n');
            list << line.c_str();
        }

        model->setStringList(list);
        this->ui->listView->setModel(model);
        this->ui->listView->update();
    }
}
