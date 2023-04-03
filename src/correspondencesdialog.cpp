#include "correspondencesdialog.h"
#include "ui_correspondencesdialog.h"

#include <qmessagebox.h>
#include <vtkRenderWindow.h>

vtkStandardNewMacro(CorrespondencesSelectionStyle)

CorrespondencesDialog::CorrespondencesDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CorrespondencesDialog)
{
    ui->setupUi(this);
    this->templateSelectionStyle = vtkSmartPointer<CorrespondencesSelectionStyle>::New();
    this->fragmentSelectionStyle = vtkSmartPointer<CorrespondencesSelectionStyle>::New();
}

CorrespondencesDialog::~CorrespondencesDialog()
{
    templateMesh->setCanvas(originalTemplateCanvas);
    fragmentMesh->setCanvas(originalFragmentCanvas);
    templateMesh = nullptr;
    fragmentMesh = nullptr;
    delete ui;
}

DrawableMesh *CorrespondencesDialog::getTemplateMesh() const
{
    return templateMesh;
}

void CorrespondencesDialog::setTemplateMesh(DrawableMesh *value)
{
    templateMesh = value;
    originalTemplateCanvas = templateMesh->getCanvas();
}

DrawableMesh *CorrespondencesDialog::getFragmentMesh() const
{
    return fragmentMesh;
}

void CorrespondencesDialog::setFragmentMesh(DrawableMesh *value)
{
    fragmentMesh = value;
    originalFragmentCanvas = fragmentMesh->getCanvas();
}

void CorrespondencesDialog::updateView()
{
    vtkSmartPointer<vtkPropAssembly> templateAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkRenderer> templateRenderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkPropAssembly> fragmentAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkRenderer> fragmentRenderer = vtkSmartPointer<vtkRenderer>::New();
    templateRenderer->SetRenderWindow(this->ui->templateWidget->renderWindow());
    templateRenderer->GetRenderWindow()->Render();
    templateRenderer->SetBackground(1.0, 1.0, 1.0);
    fragmentRenderer->SetRenderWindow(this->ui->fragmentWidget->renderWindow());
    fragmentRenderer->GetRenderWindow()->Render();
    fragmentRenderer->SetBackground(1.0, 1.0, 1.0);
    templateMesh->draw(templateAssembly);
    fragmentMesh->draw(fragmentAssembly);
    templateRenderer->AddActor(templateAssembly);
    templateRenderer->Modified();
    fragmentRenderer->AddActor(fragmentAssembly);
    fragmentRenderer->Modified();
    this->ui->templateWidget->renderWindow()->AddRenderer(templateRenderer);
    this->ui->fragmentWidget->renderWindow()->AddRenderer(fragmentRenderer);
    this->templateSelectionStyle->setMesh(templateMesh);
    this->fragmentSelectionStyle->setMesh(fragmentMesh);
    this->templateSelectionStyle->setQvtkWidget(this->ui->templateWidget);
    this->fragmentSelectionStyle->setQvtkWidget(this->ui->fragmentWidget);
    this->templateSelectionStyle->setRen(templateRenderer);
    this->fragmentSelectionStyle->setRen(fragmentRenderer);
    this->ui->templateWidget->interactor()->SetInteractorStyle(templateSelectionStyle);
    this->ui->fragmentWidget->interactor()->SetInteractorStyle(fragmentSelectionStyle);
    this->ui->templateWidget->update();
    this->ui->fragmentWidget->update();
}

void CorrespondencesDialog::on_buttonBox_accepted()
{
    std::vector<int> templateSelectedPoints = templateSelectionStyle->getPickedVertices();
    std::vector<int> fragmentSelectedPoints = fragmentSelectionStyle->getPickedVertices();
    if(templateSelectedPoints.size() == fragmentSelectedPoints.size())
        if(templateSelectedPoints.size() >= 3){
            for(unsigned int i = 0; i < templateSelectedPoints.size(); i++)
                this->correspondences.push_back(std::make_pair(templateSelectedPoints[i], fragmentSelectedPoints[i]));
            this->close();
        }else{
            QMessageBox* messageBox = new QMessageBox();
            messageBox->setText("The registration requires at least 3 points.");
            messageBox->setWindowTitle("Error");
            messageBox->show();
        }
    else{
        QMessageBox* messageBox = new QMessageBox();
        messageBox->setText("The same number of points on both the template and the fragment is required.");
        messageBox->setWindowTitle("Error");
        messageBox->show();
    }
}

void CorrespondencesDialog::on_buttonBox_rejected()
{
    this->close();
}

std::vector<std::pair<int, int> > CorrespondencesDialog::getCorrespondences() const
{
    return correspondences;
}
