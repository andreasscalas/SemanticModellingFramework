#include "attributewidget.h"
#include "ui_attributewidget.h"

#include <QCheckBox>
#include <QLabel>
#include <QVBoxLayout>
#include <geometricattribute.h>

AttributeWidget::AttributeWidget(DrawableAttribute* attribute, QTreeWidget* a_pParent,
                                 QTreeWidgetItem* a_pItem) :
    QWidget(a_pParent),
    ui(new Ui::AttributeWidget)
{
    m_pItem = a_pItem;
    ui->setupUi(this);
    connect(this, SIGNAL(pressed()), this, SLOT(ExpansionRequired()));

    QFrame* pFrame = new QFrame(this);
    QBoxLayout* pLayout = new QVBoxLayout(pFrame);
    std::stringstream buffer;
//    pLayout->addWidget(new QLabel("id: " + QString::number(attribute->getId())));
//    pLayout->addWidget(new QLabel("key: " + QString::fromStdString(attribute->getKey())));
    attribute->print(buffer);
    std::string line;

    while (std::getline(buffer, line))
        pLayout->addWidget(new QLabel(QString::fromStdString(line)));
    pLayout->addWidget(new QCheckBox("Draw attribute"));


}

AttributeWidget::~AttributeWidget()
{
    delete ui;
}

DrawableAttribute *AttributeWidget::getAttribute() const
{
    return attribute;
}

void AttributeWidget::setAttribute(DrawableAttribute *value)
{
    attribute = value;
}

void AttributeWidget::ExpansionRequired()
{
    m_pItem->setExpanded( !m_pItem->isExpanded() );
}
