#include "annotationwidget.h"
#include "ui_annotationwidget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <attributewidget.h>

AnnotationWidget::AnnotationWidget(DrawableAnnotation* annotation, QTreeWidget* a_pParent,
                                   QTreeWidgetItem* a_pItem) :
    QWidget(a_pParent),
    ui(new Ui::AnnotationWidget)
{
    m_pItem = a_pItem;
    ui->setupUi(this);
    connect(this, SIGNAL(pressed()), this, SLOT(ExpansionRequired()));

    QFrame* pFrame = new QFrame(this);
    QBoxLayout* pLayout = new QVBoxLayout(pFrame);
    pLayout->addWidget(new QLabel("id: "+QString::number(annotation->getId())));
    pLayout->addWidget(new QLabel("tag: "+QString::fromStdString(annotation->getTag())));
    for(unsigned int i = 0; i < annotation->getAttributes().size(); i++)
    {
        Attribute* attr = annotation->getAttributes()[i];
        QTreeWidgetItem* attributeWidget = new QTreeWidgetItem();
        a_pParent->addTopLevelItem(attributeWidget);
        a_pParent->setItemWidget(attributeWidget, 1,
            new AttributeWidget(dynamic_cast<DrawableAttribute*>(attr), a_pParent, attributeWidget));
    }


    QTreeWidgetItem* pContainer = new QTreeWidgetItem();
    pContainer->setDisabled(true);
    a_pItem->addChild(pContainer);
    a_pParent->setItemWidget(pContainer, 1, pFrame);
}

AnnotationWidget::~AnnotationWidget()
{
    delete ui;
}

void AnnotationWidget::ExpansionRequired()
{
    m_pItem->setExpanded( !m_pItem->isExpanded() );
}
