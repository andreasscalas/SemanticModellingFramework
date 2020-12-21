#include "geometricattributedialog.h"
#include "ui_geometricattributedialog.h"
#include <QInputDialog>
#include <QListView>
#include <QMenu>
#include <QMouseEvent>
#include <geometricattribute.h>
#include <QDir>


vtkStandardNewMacro(MeasureStyle)

GeometricAttributeDialog::GeometricAttributeDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GeometricAttributeDialog)
{
    ui->setupUi(this);
    connect(this->ui->rulerRadioButton, SIGNAL(toggled(bool)), this, SLOT(rulerSelected(bool)));
    connect(this->ui->tapeRadioButton, SIGNAL(toggled(bool)), this, SLOT(tapeSelected(bool)));
    connect(this->ui->caliberRadioButton, SIGNAL(toggled(bool)), this, SLOT(caliperSelected(bool)));
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);
    renderer->SetLayer(0);
    renderer->AddActor(canvas);
    measureStyle = vtkSmartPointer<MeasureStyle>::New();
    measureStyle->setMeshRenderer(this->renderer);
    measureStyle->setQvtkwidget(this->ui->qvtkWidget);
    this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);

}

GeometricAttributeDialog::~GeometricAttributeDialog()
{
    mesh = nullptr;
    delete ui;
}

void GeometricAttributeDialog::updateView()
{

    mesh->setDrawAnnotations(true);
    mesh->update();
    mesh->draw(canvas);
    measureStyle->setMesh(mesh);
    this->ui->qvtkWidget->update();
}

void GeometricAttributeDialog::rulerSelected(bool isSelected)
{
    if(isSelected)
        measureStyle->setMeasureType(MeasureStyle::MeasureType::RULER);
}

void GeometricAttributeDialog::tapeSelected(bool isSelected)
{
    if(isSelected)
        measureStyle->setMeasureType(MeasureStyle::MeasureType::TAPE);
}

void GeometricAttributeDialog::caliperSelected(bool isSelected)
{
    if(isSelected)
        measureStyle->setMeasureType(MeasureStyle::MeasureType::CALIBER);
}

DrawableMesh *GeometricAttributeDialog::getMesh() const
{
    return mesh;
}

void GeometricAttributeDialog::setMesh(DrawableMesh *value)
{
    mesh = (value);
}

void GeometricAttributeDialog::on_addMeasureButton_clicked()
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
                                             tr("Attribute name:"), QLineEdit::Normal,
                                             QDir::home().dirName(), &ok);
    if (ok && !text.isEmpty()){
        for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++){
            if(mesh->getDAnnotations()[i]->getSelected()){
                Annotation* ann = mesh->getAnnotations()[i];
                GeometricAttribute* attribute = new GeometricAttribute();
                attribute->setKey(text.toStdString());
                double measure = measureStyle->getMeasure();
                attribute->setValue(&measure);
                if(this->ui->tapeRadioButton->isChecked()){
                    std::vector<IMATI_STL::Vertex*> measurePoints;
                    for (unsigned int i = 0; i < measurePoints.size(); i++)
                        attribute->addMeasurePointID(mesh->getPointId(measurePoints[i]));
                }

                ann->addAttribute(attribute);
            }
        }
    }


}
