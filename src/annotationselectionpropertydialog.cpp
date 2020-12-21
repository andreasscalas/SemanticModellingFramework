#include "annotationselectionpropertydialog.h"
#include "ui_annotationselectionpropertydialog.h"

#include <geometricattributedialog.h>

AnnotationSelectionPropertyDialog::AnnotationSelectionPropertyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AnnotationSelectionPropertyDialog)
{
    ui->setupUi(this);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);
    renderer->SetLayer(0);
    renderer->AddActor(canvas);
    annotationStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    annotationStyle->setRen(renderer);
    annotationStyle->setAssembly(canvas);
    annotationStyle->setQvtkWidget(this->ui->qvtkWidget);
    this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(annotationStyle);
}

AnnotationSelectionPropertyDialog::~AnnotationSelectionPropertyDialog()
{
    delete mesh;
    delete ui;
}

void AnnotationSelectionPropertyDialog::updateView()
{
    mesh->update();
    mesh->draw(canvas);
    this->ui->qvtkWidget->update();
}

void AnnotationSelectionPropertyDialog::on_buttonBox_accepted()
{
    GeometricAttributeDialog* gad = new GeometricAttributeDialog(parentWidget());
    gad->setMesh(mesh);
    gad->updateView();
    gad->show();
    close();

}

void AnnotationSelectionPropertyDialog::on_buttonBox_rejected()
{
    this->close();
}

DrawableMesh *AnnotationSelectionPropertyDialog::getMesh() const
{
    return mesh;
}

void AnnotationSelectionPropertyDialog::setMesh(DrawableMesh *value)
{
    mesh = new DrawableMesh(value);
    annotationStyle->setMesh(mesh);
    annotationStyle->resetSelection();
}
