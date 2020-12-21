#include "measureslistwidget.h"
#include "ui_measureslistwidget.h"

#include <QPushButton>
#include <QVBoxLayout>
#include <QHeaderView>
#include <annotationwidget.h>

MeasuresListWidget::MeasuresListWidget(QWidget *parent) :
    QTreeWidget(parent),
    ui(new Ui::MeasuresListWidget)
{
    ui->setupUi(this);

    this->setRootIsDecorated(false);
    this->setIndentation(0);
}

MeasuresListWidget::~MeasuresListWidget()
{
    delete ui;
}

void MeasuresListWidget::update()
{
//    this->clear();

//    for(unsigned int i = 0; i < mesh->getDAnnotations().size(); i++)
//    {
//        QTreeWidgetItem* pCategory = new QTreeWidgetItem();
//        this->addTopLevelItem(pCategory);
//        this->setItemWidget(pCategory, 0,
//            new AnnotationWidget(mesh->getDAnnotations()[i], this, pCategory));

//    }
}

DrawableMesh *MeasuresListWidget::getMesh() const
{
    return mesh;
}

void MeasuresListWidget::setMesh(DrawableMesh *value)
{
    mesh = value;
}
