#include "annotationwindow.h"
#include "ui_annotationwindow.h"
#include <geometricattribute.h>

#include <annotationfilemanager.h>
#include <semanticattribute.h>
#include <semanticattributedialog.h>
#include <semanticgraphinteractionstyle.h>
#include <relationshipsdialog.h>
#include <annotationmeasuresconstraint.h>
#include <drawableareaannotation.h>
#include <drawablepointannotation.h>
#include <utilities.h>
#include <annotationutilities.h>

#include <vtkIdFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <vtkAreaPicker.h>

#include <QVTKOpenGLNativeWidget.h>

#include <QInputDialog>
#include <QMenu>
#include <QDir>
#include <QMessageBox>
#include <QFileDialog>


vtkStandardNewMacro(MeasureStyle)
vtkStandardNewMacro(AnnotationSelectionInteractorStyle)
vtkStandardNewMacro(VerticesSelectionStyle)
vtkStandardNewMacro(TriangleSelectionStyle)
vtkStandardNewMacro(LineSelectionStyle)
AnnotationWindow::AnnotationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AnnotationWindow)
{
    ui->setupUi(this);

    ad = new AnnotationDialog(this);
    cd = new AnnotationConstraintDialog(this);
    connect(ui->actionSaveAnnotation, SIGNAL(triggered(bool)), this, SLOT(slotSaveAnnotation()));
    connect(ui->actionOpenAnnotation, SIGNAL(triggered(bool)), this, SLOT(slotOpenAnnotation()));
    connect(ui->actionClearAnnotation, SIGNAL(triggered(bool)), this, SLOT(slotClearAnnotation()));
    connect(ui->actionClearSelections, SIGNAL(triggered(bool)), this, SLOT(slotClearSelections()));
    connect(ui->actionCamera, SIGNAL(triggered(bool)), this, SLOT(slotCamera(bool)));
    connect(ui->actionResetCamera, SIGNAL(triggered(bool)), this, SLOT(slotResetCamera(bool)));
    connect(ui->actionVisible, SIGNAL(triggered(bool)), this, SLOT(slotVisible(bool)));
    connect(ui->actionDeleteMode, SIGNAL(triggered(bool)), this, SLOT(slotDeleteMode(bool)));
    connect(ui->actionVerticesSelection, SIGNAL(triggered(bool)), this, SLOT(slotVerticesSelection(bool)));
    connect(ui->actionLinesSelection, SIGNAL(triggered(bool)), this, SLOT(slotLinesSelection(bool)));
    connect(ui->actionTrianglesInRectangle, SIGNAL(triggered(bool)), this, SLOT(slotTrianglesInRectangle(bool)));
    connect(ui->actionTrianglesInPolygon, SIGNAL(triggered(bool)), this, SLOT(slotTrianglesInPolygon(bool)));
    connect(ui->actionAnnotate, SIGNAL(triggered(bool)), this, SLOT(slotAnnotate()));
    connect(ui->actionSelectAnnotation, SIGNAL(triggered(bool)), this, SLOT(slotSelectAnnotation(bool)));
    connect(ui->actionShowRelationships, SIGNAL(triggered(bool)), this, SLOT(slotShowRelationships()));
    connect(ui->actionBuildRelationshipsGraph, SIGNAL(triggered(bool)), this, SLOT(slotBuildRelationshipsGraph()));
    connect(ui->actionReleaseConstraints, SIGNAL(triggered(bool)), this, SLOT(slotReleaseConstraints()));
    connect(ui->actionEditAnnotation, SIGNAL(triggered(bool)), this, SLOT(editAnnotation(bool)));
    connect(ui->rulerButton, SIGNAL(toggled(bool)), this, SLOT(rulerSelected(bool)));
    connect(ui->tapeMeasureButton, SIGNAL(toggled(bool)), this, SLOT(tapeSelected(bool)));
    connect(ui->boundingButton, SIGNAL(toggled(bool)), this, SLOT(boundingSelected(bool)));
    connect(ui->heightButton, SIGNAL(toggled(bool)), this, SLOT(heightSelected(bool)));
    connect(ad, SIGNAL(finalizationCalled(std::string, uchar*)), SLOT(slotFinalization(std::string, uchar*)));
    connect(ui->measuresList, SIGNAL(updateSignal()), this, SLOT(slotUpdate()));
    connect(ui->measuresList, SIGNAL(updateViewSignal()), this, SLOT(slotUpdateView()));
    connect(this->ui->actionTransfer, SIGNAL(triggered(bool)), this, SLOT(slotTransfer()));
    connect(this->ui->actionConstrain, SIGNAL(triggered()), this, SLOT(slotConstrain()));
    connect(this->ui->actionAddConstraint, SIGNAL(triggered()), this, SLOT(slotActionAddConstraint()));
    connect(cd, SIGNAL(addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsConstraint(std::string, double, double, double, unsigned int, unsigned int, bool)));

    reachedId = 0;
    coreGraphExtracted = false;
    isAnnotationBeingModified = false;
    coordsComputed = false;
    isSystemConstrained = false;
    mesh = nullptr;
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetActiveCamera(vtkSmartPointer<vtkCamera>::New());
    renderer->SetBackground(1.0, 1.0, 1.0);
    renderer->AddActor(canvas);
    annotationStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    annotationStyle->setRen(renderer);
    annotationStyle->setAssembly(canvas);
    annotationStyle->setQvtkWidget(ui->qvtkWidget);
    measureAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    measureStyle = vtkSmartPointer<MeasureStyle>::New();
    measureStyle->setMeshRenderer(renderer);
    measureStyle->setQvtkwidget(this->ui->qvtkWidget);
    this->ui->actionConstrain->setEnabled(false);
    this->ui->actionReleaseConstraints->setEnabled(false);
    ui->qvtkWidget->renderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
    vtkSmartPointer<vtkAreaPicker> picker = vtkSmartPointer<vtkAreaPicker>::New();
    this->ui->qvtkWidget->renderWindow()->GetInteractor()->SetPicker(picker);
    this->initialCamera = vtkSmartPointer<vtkCamera>::New();
    this->initialCamera->DeepCopy(renderer->GetActiveCamera());
}

AnnotationWindow::~AnnotationWindow()
{
    delete ui;
}

void AnnotationWindow::annotationsModified()
{

    this->ui->measuresList->setMesh(mesh);
    this->ui->measuresList->update();
}

void AnnotationWindow::updateView()
{
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->setRenderer(renderer);
    mesh->draw(canvas);
    ui->qvtkWidget->update();
}


void AnnotationWindow::rulerSelected(bool isSelected)
{
    if(isSelected){
        measureStyle->setDrawAttributes(true);
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::RULER);
    }
}

void AnnotationWindow::tapeSelected(bool isSelected)
{
    if(isSelected){
        measureStyle->setDrawAttributes(true);
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::TAPE);
    }
}

void AnnotationWindow::caliperSelected(bool isSelected)
{
    if(isSelected){
        measureStyle->setDrawAttributes(true);
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::CALIBER);
    }
}

void AnnotationWindow::boundingSelected(bool isSelected)
{
    if(isSelected){
        measureStyle->setDrawAttributes(true);
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::BOUNDING);
    }
}

void AnnotationWindow::heightSelected(bool isSelected)
{
    if(isSelected){
        measureStyle->setDrawAttributes(true);
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::HEIGHT);
    }
}

void AnnotationWindow::editAnnotation(bool)
{
    if(annotationStyle->getSelectedAnnotations().size() == 1){
        this->isAnnotationBeingModified = true;
        renderer->RemoveActor(canvas);
        annotationBeingModified = annotationStyle->getSelectedAnnotations()[0];
        annotationStyle->resetSelection();
        annotationStyle->modifySelectedAnnotations();

        if(annotationBeingModified->getType() == AnnotationType::Surface){
            std::vector<IMATI_STL::Triangle*> selectedTriangles = dynamic_cast<SurfaceAnnotation*>(annotationBeingModified)->getTriangles();
            trianglesSelectionStyle->setAssembly(canvas);
            trianglesSelectionStyle->defineSelection(Utilities::trianglesToIDvector(mesh, selectedTriangles));
            trianglesSelectionStyle->modifySelectedTriangles();
            this->ui->actionTrianglesInRectangle->trigger();
        }

        mesh->removeAnnotation(annotationBeingModified);
        //mesh->removeAnnotation(dynamic_cast<DrawableAnnotation*>(mesh->getAnnotations()[annotationBeingModified->getId()]));
        mesh->draw(this->canvas);
        this->canvas->Modified();
        renderer->AddActor(canvas);
        renderer->Modified();
        annotationsModified();
        this->ui->qvtkWidget->update();
    } else {
        QMessageBox* dialog = new QMessageBox(this);
        dialog->setWindowTitle("Error");
        dialog->setText("This action can be applied only if exactly one annotation is selected");
        dialog->show();
    }
}

void AnnotationWindow::openContextualMenu(vtkObject *obj, unsigned long, void *, void *)
{
    obj->Print(std::cout);
    std::cout << std::flush;
    QMenu contextMenu(tr("Context menu"), this);

    QAction action1("Remove Data Point", this);
    contextMenu.addAction(&action1);

    contextMenu.exec();
}

DrawableMesh *AnnotationWindow::getMesh() const
{
    return mesh;
}

void AnnotationWindow::setMesh(DrawableMesh *value)
{
    mesh = value;
    mesh->setDrawAnnotations(true);
    mesh->draw(canvas);

    /*for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
        {
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->update();
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->setDrawAttribute(true);
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->setRenderer(renderer);
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->draw(canvas);
        }*/
    if(mesh->getGraph() != nullptr)
    {
        coreGraphExtracted = true;
        if(!isSystemConstrained && coordsComputed)
        {
            this->ui->actionConstrain->setEnabled(true);
            this->ui->actionReleaseConstraints->setEnabled(false);
        }
        else if(isSystemConstrained)
        {
            this->ui->actionConstrain->setEnabled(false);
            this->ui->actionReleaseConstraints->setEnabled(true);
        }
    }


    reachedId = static_cast<unsigned int>(mesh->getAnnotations().size());

    if(mesh->getAnnotations().size() > 0)
        annotationsModified();
    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(mesh->getMeshSurfaceActor()->GetMapper()->GetInputAsDataSet());
    idFilter->PointIdsOn();
    idFilter->SetCellIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkSmartPointer<vtkIdFilter> idFilterMesh = vtkSmartPointer<vtkIdFilter>::New();
    idFilterMesh->SetInputData(mesh->getMeshPointsActor()->GetMapper()->GetInputAsDataSet());
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetPointIdsArrayName("OriginalMeshIds");
    idFilterMesh->Update();

    vtkSmartPointer<vtkPolyData> input = static_cast<vtkPolyData*>(idFilter->GetOutput());
    vtkSmartPointer<vtkPolyData> inputMesh = static_cast<vtkPolyData*>(idFilterMesh->GetOutput());

    verticesSelectionStyle = vtkSmartPointer<VerticesSelectionStyle>::New();
    verticesSelectionStyle->setPoints(inputMesh);
    verticesSelectionStyle->setAssembly(canvas);
    verticesSelectionStyle->setMesh(mesh);
    verticesSelectionStyle->setPointsSelectionStatus(&(pointsSelectionStatus));
    verticesSelectionStyle->setQvtkwidget(ui->qvtkWidget);
    verticesSelectionStyle->resetSelection();

    linesSelectionStyle = vtkSmartPointer<LineSelectionStyle>::New();
    linesSelectionStyle->setRen(renderer);
    linesSelectionStyle->setPoints(input);
    linesSelectionStyle->setMesh(mesh);
    linesSelectionStyle->setAssembly(canvas);
    linesSelectionStyle->setEdgesSelectionStatus(&(edgeSelectionStatus));
    linesSelectionStyle->setQvtkwidget(ui->qvtkWidget);
    linesSelectionStyle->resetSelection();

    trianglesSelectionStyle = vtkSmartPointer<TriangleSelectionStyle>::New();
    trianglesSelectionStyle->SetTriangles(input);
    trianglesSelectionStyle->setMesh(mesh);
    trianglesSelectionStyle->setAssembly(canvas);
    trianglesSelectionStyle->setTrianglesSelectionStatus(&(triangleSelectionStatus));
    trianglesSelectionStyle->setRen(renderer);
    trianglesSelectionStyle->setQvtkWidget(ui->qvtkWidget);
    trianglesSelectionStyle->resetSelection();

    annotationStyle->setMesh(mesh);
    annotationStyle->resetSelection();

    measureStyle->setMesh(mesh);
    measureStyle->setMeasureType(MeasureStyle::MeasureType::RULER);

    this->ui->qvtkWidget->update();
    this->ui->measuresList->setMesh(mesh);
    renderer->ResetCamera();

    originalCamera = vtkSmartPointer<vtkCamera>::New();
    originalCamera->DeepCopy(this->renderer->GetActiveCamera());

}

void AnnotationWindow::slotSaveAnnotation()
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the annotations on the mesh",
                       QString::fromStdString(currentPath),
                       "ANT(*.ant);;FCT(*.fct);;TRIANT(*.triant);;M(*.m)");

    if (!filename.isEmpty()){

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        emit currentPathChanged(currentPath);
        AnnotationFileManager manager;
        manager.setMesh(mesh);
        if(!manager.writeAnnotations(filename.toStdString()))
            std::cout << "Something went wrong during annotation file writing." << std::endl << std::flush;
    }
}

void AnnotationWindow::slotOpenAnnotation()
{
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Choose an annotation file",
                       QString::fromStdString(currentPath),
                       "ANT(*.ant);;FCT(*.fct);;TRIANT(*.triant);;All(*.*)");

    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        emit currentPathChanged(currentPath);
        AnnotationFileManager manager;
        manager.setMesh(mesh);
        if(!manager.readAnnotations(filename.toStdString()))
            std::cout<<"Something went wrong during annotation file opening."<< std::endl<< std::flush;
        reachedId = mesh->getAnnotations().size();
        for(unsigned int i = 0; i < reachedId; i++)
        {
            DrawableAnnotation* dAnnotation;
            Annotation* a = mesh->getAnnotations()[i];
            if(a->getType() == AnnotationType::Surface)
               dAnnotation = new DrawableAreaAnnotation(dynamic_cast<SurfaceAnnotation*>(a));
            else if(a->getType() == AnnotationType::Line){
               dAnnotation = new DrawableLineAnnotation(dynamic_cast<LineAnnotation*>(a));
               static_cast<DrawableLineAnnotation*>(dAnnotation)->setLineWidth(10);
            } else if(a->getType() == AnnotationType::Point){
               dAnnotation = new DrawablePointAnnotation(dynamic_cast<PointAnnotation*>(a));
            }
            dAnnotation->setMeshPoints(mesh->getPoints());
            dAnnotation->update();
            mesh->setAnnotation(i, dAnnotation);
        }
        annotationsModified();
        mesh->setAnnotationsModified(true);
        mesh->update();
        mesh->draw(canvas);
        this->ui->measuresList->update();
        this->ui->qvtkWidget->update();
    }
}

void AnnotationWindow::slotClearAnnotation()
{
    mesh->clearAnnotations();
    mesh->setAnnotationsModified(true);
    mesh->update();
    mesh->draw(canvas);
    annotationsModified();
    this->ui->qvtkWidget->update();
}

void AnnotationWindow::slotClearSelections()
{
    verticesSelectionStyle->resetSelection();
    verticesSelectionStyle->modifySelectedPoints();
    linesSelectionStyle->resetSelection();
    linesSelectionStyle->modifySelectedLines();
    trianglesSelectionStyle->resetSelection();
    trianglesSelectionStyle->modifySelectedTriangles();
}

void AnnotationWindow::slotResetCamera(bool)
{
    vtkSmartPointer<vtkCamera> newActiveCamera = vtkSmartPointer<vtkCamera>::New();
    newActiveCamera->DeepCopy(originalCamera);
    this->renderer->SetActiveCamera(newActiveCamera);
    this->renderer->ResetCamera();
    this->ui->qvtkWidget->update();
}

void AnnotationWindow::slotCamera(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> cameraStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        ui->qvtkWidget->interactor()->SetInteractorStyle(cameraStyle);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
    } else
        ui->actionCamera->setChecked(true);

}

void AnnotationWindow::slotVisible(bool checked)
{
    this->verticesSelectionStyle->setVisiblePointsOnly(checked);
    this->linesSelectionStyle->setVisiblePointsOnly(checked);
    this->trianglesSelectionStyle->setVisibleTrianglesOnly(checked);
}

void AnnotationWindow::slotDeleteMode(bool checked)
{
    if(checked){
        verticesSelectionStyle->setSelectionMode(false);
        trianglesSelectionStyle->setSelectionMode(false);
    }else{
        verticesSelectionStyle->setSelectionMode(true);
        trianglesSelectionStyle->setSelectionMode(true);
    }
}

void AnnotationWindow::slotVerticesSelection(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        ui->qvtkWidget->interactor()->SetInteractorStyle(verticesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(false);
        mesh->setDrawSurface(true);
        mesh->draw(canvas);
        linesSelectionStyle->resetSelection();
        trianglesSelectionStyle->resetSelection();
        ui->qvtkWidget->update();
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->rulerButton->setAutoExclusive(false);
        ui->tapeMeasureButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotLinesSelection(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        ui->qvtkWidget->interactor()->SetInteractorStyle(linesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(false);
        mesh->setDrawSurface(true);
        mesh->draw(canvas);
        verticesSelectionStyle->resetSelection();
        trianglesSelectionStyle->resetSelection();
        ui->qvtkWidget->update();
        ui->actionVerticesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->rulerButton->setAutoExclusive(false);
        ui->tapeMeasureButton->setAutoExclusive(false);
        //ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        //ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        //ui->caliberButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotTrianglesInRectangle(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        trianglesSelectionStyle->setSelectionType(TriangleSelectionStyle::RECTANGLE_AREA);
        ui->qvtkWidget->interactor()->SetInteractorStyle(trianglesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(false);
        mesh->setDrawSurface(true);
        mesh->draw(canvas);
        verticesSelectionStyle->resetSelection();
        linesSelectionStyle->resetSelection();
        ui->qvtkWidget->update();
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->rulerButton->setAutoExclusive(false);
        ui->tapeMeasureButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotTrianglesInPolygon(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        trianglesSelectionStyle->setSelectionType(TriangleSelectionStyle::LASSO_AREA);
        ui->qvtkWidget->interactor()->SetInteractorStyle(trianglesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(false);
        mesh->setDrawSurface(true);
        mesh->draw(canvas);
        ui->qvtkWidget->update();
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->rulerButton->setAutoExclusive(false);
        ui->tapeMeasureButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotAnnotate()
{
    ad->show();
}

void AnnotationWindow::slotFinalization(std::string tag, uchar* color){
    unsigned int id;
    if(isAnnotationBeingModified){
        id = annotationBeingModified->getId();
        isAnnotationBeingModified = false;
        annotationBeingModified = nullptr;
    }else
        id = reachedId++;

    annotationsModified();
    if(ui->qvtkWidget->interactor()->GetInteractorStyle()->IsA("TriangleSelectionStyle"))
        trianglesSelectionStyle->finalizeAnnotation(id, tag, color);
    else if(ui->qvtkWidget->interactor()->GetInteractorStyle()->IsA("LineSelectionStyle"))
        linesSelectionStyle->finalizeAnnotation(id, tag, color);
    else if(ui->qvtkWidget->interactor()->GetInteractorStyle()->IsA("VerticesSelectionStyle"))
        verticesSelectionStyle->finalizeAnnotation(id, tag, color);
    this->ui->measuresList->update();
    this->ui->qvtkWidget->update();
}



void AnnotationWindow::slotConstrain()
{
    emit constrainSystem(semanticConstraints);
    this->isSystemConstrained = true;
    this->ui->actionConstrain->setEnabled(false);
    this->ui->actionReleaseConstraints->setEnabled(true);
}

void AnnotationWindow::slotReleaseConstraints()
{
    emit(releaseConstraints());

    this->isSystemConstrained = false;

    if(coreGraphExtracted && coordsComputed)
        this->ui->actionConstrain->setEnabled(true);
    else
        this->ui->actionConstrain->setEnabled(false);
    this->ui->actionReleaseConstraints->setEnabled(false);
}

void AnnotationWindow::slotUpdate()
{
    this->mesh->setAnnotationsModified(true);
    this->mesh->update();
    updateView();
    measureStyle->updateView();
}

void AnnotationWindow::slotUpdateView()
{
    updateView();
    measureStyle->updateView();
}

void AnnotationWindow::slotSelectAnnotation(bool checked)
{
    if(checked){
        measureStyle->setDrawAttributes(true);
        ui->qvtkWidget->interactor()->SetInteractorStyle(annotationStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(false);
        mesh->setDrawSurface(true);
        mesh->setDrawAnnotations(true);
        mesh->draw(canvas);
        ui->qvtkWidget->update();
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->rulerButton->setAutoExclusive(false);
        ui->tapeMeasureButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);

}

void AnnotationWindow::slotBuildRelationshipsGraph()
{
    GraphTemplate::Graph<Annotation*>* graph = Utilities::buildRelationshipsGraph(mesh->getAnnotations());
    mesh->setGraph(graph);
    coreGraphExtracted = true;
    if(coordsComputed)
        this->ui->actionConstrain->setEnabled(true);
}

void AnnotationWindow::slotShowRelationships()
{
    RelationshipsDialog* d = new RelationshipsDialog(this);
    d->setMesh(mesh);
    d->setIsSystemConstrainable(coordsComputed && !isSystemConstrained);
    d->update();
    d->updateView();
    d->show();
    connect(d, SIGNAL(addSemanticRelationship(std::vector<Annotation*>, std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsRelationship(std::vector<Annotation*>, std::string, double, double, double, unsigned int, unsigned int, bool)));
    connect(d, SIGNAL(constrainRelationship(AnnotationsRelationship*&)), this, SLOT(slotConstrainRelationship(AnnotationsRelationship*&)));
    connect(d, SIGNAL(constrainRelationships()), this, SLOT(slotConstrain()));
    connect(d, SIGNAL(releaseConstraints()), this, SLOT(slotReleaseConstraints()));

}

void AnnotationWindow::on_addMeasureButton_clicked()
{
    std::vector<Annotation*> selected = annotationStyle->getSelectedAnnotations();

    if(selected.size() > 0){
        bool ok;
        QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
                                                 tr("Attribute name:"), QLineEdit::Normal,
                                                 QDir::home().dirName(), &ok);
        if (ok && !text.isEmpty()){
            DrawableAttribute* attribute = measureStyle->finalizeAttribute(selected[0]->getAttributes().size(), text.toStdString());
            selected[0]->addAttribute(attribute);
        }
        annotationsModified();
    } else {
        QMessageBox* dialog = new QMessageBox(this);
        dialog->setWindowTitle("Error");
        dialog->setText("You need to select at least one annotation");
        dialog->show();
    }
}

void AnnotationWindow::on_addPropertyButton_clicked()
{
    std::vector<Annotation*> selected;
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        if(dynamic_cast<DrawableAnnotation*>(mesh->getAnnotations()[i])->getSelected())
            selected.push_back(mesh->getAnnotations()[i]);

    if(selected.size() > 0){
        SemanticAttributeDialog* saDialog = new SemanticAttributeDialog(this);
        saDialog->exec();
        QString name, value;
        if(saDialog->getSuccess()){
            name = saDialog->getAttributeName();
            value = saDialog->getAttributeValue();

            if (!name.isEmpty()){
                for(unsigned int i = 0; i < selected.size(); i++){
                    SemanticAttribute* attribute = new SemanticAttribute();
                    attribute->setKey(name.toStdString());
                    attribute->setValue(value.toStdString());
                    selected[i]->addAttribute(attribute);
                }
            }
        }
        annotationsModified();

    } else {
        QMessageBox* dialog = new QMessageBox(this);
        dialog->setWindowTitle("Error");
        dialog->setText("You need to select at least one annotation");
        dialog->show();
    }
}

std::string AnnotationWindow::getCurrentPath() const
{
    return currentPath;
}

void AnnotationWindow::setCurrentPath(const std::string &value)
{
    currentPath = value;
}

bool AnnotationWindow::areCoordsComputed() const
{
    return coordsComputed;
}

void AnnotationWindow::setCoordsComputed(bool value)
{
    coordsComputed = value;
}

ShapeOp::MatrixX3 *AnnotationWindow::getSoMeshPoints() const
{
    return soMeshPoints;
}

void AnnotationWindow::setSoMeshPoints(ShapeOp::MatrixX3 *value)
{
    soMeshPoints = value;
}

bool AnnotationWindow::getIsSystemConstrained() const
{
    return isSystemConstrained;
}

void AnnotationWindow::setIsSystemConstrained(bool value)
{
    isSystemConstrained = value;
}

std::vector<AnnotationsConstraint *> AnnotationWindow::getSemanticConstraints() const
{
    return semanticConstraints;
}

void AnnotationWindow::setSemanticConstraints(const std::vector<AnnotationsConstraint *> &value)
{
    semanticConstraints = value;
}

ConstraintSolver *AnnotationWindow::getSolver() const
{
    return solver;
}

void AnnotationWindow::setSolver(ConstraintSolver *value)
{
    solver = value;
}

void AnnotationWindow::closeEvent(QCloseEvent *event)
{
    for(unsigned int i = 0; i < this->mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < this->mesh->getAnnotations()[i]->getAttributes().size(); j++)
            measureAssembly->RemovePart(dynamic_cast<DrawableAttribute*>(this->mesh->getAnnotations()[i]->getAttributes()[j])->getCanvas());
    canvas->RemovePart(measureAssembly);
    canvas->RemovePart(mesh->getCanvas());
    emit(annotationWindowClosed(mesh, semanticConstraints));
}


void AnnotationWindow::slotActionAddConstraint()
{
    std::vector<Annotation*> meshAnnotations = mesh->getAnnotations();
    std::vector<Annotation*> selected = annotationStyle->getSelectedAnnotations();
    cd->setSubjects(selected);
    cd->show();
}
void AnnotationWindow::slotAddAnnotationsConstraint(std::string type, double weight, double minValue, double maxValue, unsigned int measureId1, unsigned int measureId2, bool directed)
{
    AnnotationsConstraint* constraint;
    if(type.compare("Surfaces same measure") == 0){
        Annotation* annotation1 = cd->getSubjects()[0];
        Annotation* annotation2 = cd->getSubjects()[1];
        Attribute* attribute1 = annotation1->getAttributes()[measureId1];
        Attribute* attribute2 = annotation2->getAttributes()[measureId2];
        constraint = new AnnotationMeasuresConstraint();
        GeometricAttribute* geometricAttribute1 = dynamic_cast<GeometricAttribute*>(attribute1);
        GeometricAttribute* geometricAttribute2 = dynamic_cast<GeometricAttribute*>(attribute2);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute1(geometricAttribute1);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute2(geometricAttribute2);
    }else if(type.compare("Surface same measure") == 0){
        if(measureId1 == measureId2){
            std::cerr << "Error: constraining a measure to be equal to itself is not sensible" << std::endl << std::flush;
            return;
        }
        Annotation* annotation = cd->getSubjects()[0];
        Attribute* attribute1 = annotation->getAttributes()[measureId1];
        Attribute* attribute2 = annotation->getAttributes()[measureId2];
        constraint = new AnnotationMeasuresConstraint();
        GeometricAttribute* geometricAttribute1 = dynamic_cast<GeometricAttribute*>(attribute1);
        GeometricAttribute* geometricAttribute2 = dynamic_cast<GeometricAttribute*>(attribute2);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute1(geometricAttribute1);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute2(geometricAttribute2);
    } else
        constraint = new AnnotationsConstraint();
    constraint->setId(reachedContraintId++);
    constraint->setPoints(soMeshPoints);
    constraint->setAnnotations(cd->getSubjects());
    constraint->setType(type);
    constraint->setWeight(weight);
    constraint->setMinValue(minValue);
    constraint->setMaxValue(maxValue);
    constraint->constrain();
    semanticConstraints.push_back(constraint);

    for (unsigned int i = 0; i < constraint->getAnnotations().size(); i++) {
        for (unsigned int j = i; j < constraint->getAnnotations().size(); j++) {
            if( i == j && constraint->getAnnotations().size() != 1)
                continue;
            mesh->addAnnotationsRelationship(constraint->getAnnotations()[i], constraint->getAnnotations()[j], type, weight, directed);
        }
    }


    this->mesh->draw(this->canvas);
    this->ui->qvtkWidget->update();
}

void AnnotationWindow::slotAddAnnotationsRelationship(std::vector<Annotation *> subjects, std::string type, double weight, double minValue, double maxValue, unsigned int measureId1, unsigned int measureId2, bool directed)
{
    AnnotationsConstraint* constraint;
    if(type.compare("Surfaces same measure") == 0){
        Annotation* annotation1 = subjects[0];
        Annotation* annotation2 = subjects[1];
        Attribute* attribute1 = annotation1->getAttributes()[measureId1];
        Attribute* attribute2 = annotation2->getAttributes()[measureId2];
        constraint = new AnnotationMeasuresConstraint();
        GeometricAttribute* geometricAttribute1 = dynamic_cast<GeometricAttribute*>(attribute1);
        GeometricAttribute* geometricAttribute2 = dynamic_cast<GeometricAttribute*>(attribute2);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute1(geometricAttribute1);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute2(geometricAttribute2);
    } else
        constraint = new AnnotationsConstraint();
    constraint->setId(reachedContraintId++);
    constraint->setPoints(soMeshPoints);
    constraint->setAnnotations(subjects);
    constraint->setType(type);
    constraint->setWeight(weight);
    constraint->setMinValue(minValue);
    constraint->setMaxValue(maxValue);
    constraint->constrain();
    semanticConstraints.push_back(constraint);

    for (unsigned int i = 0; i < constraint->getAnnotations().size(); i++) {
        for (unsigned int j = i; j < constraint->getAnnotations().size(); j++) {
            if( i == j && constraint->getAnnotations().size() != 1)
                continue;
            mesh->addAnnotationsRelationship(constraint->getAnnotations()[i], constraint->getAnnotations()[j], type, weight, directed);
        }
    }

    this->mesh->draw(this->canvas);
    this->ui->qvtkWidget->update();
}


void AnnotationWindow::slotConstrainRelationship(AnnotationsRelationship *&relationship)
{
    AnnotationsRelationship* r = relationship;
    AnnotationsConstraint* constraint = new AnnotationsConstraint(relationship);
    constraint->setPoints(soMeshPoints);
    constraint->constrain();
    std::cout << constraint << " " << relationship << std::endl << std::flush;
    relationship = constraint;
    delete r;
    semanticConstraints.push_back(constraint);
    this->mesh->setMeshModified(true);
    this->mesh->update();
    this->mesh->draw(this->canvas);
    this->ui->qvtkWidget->update();
}


void AnnotationWindow::slotTransfer(){

    DrawableMesh* otherModel = new DrawableMesh();
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Choose a 3D model",
                       QString::fromStdString(currentPath),
                       "STL(*.stl);;OBJ(*.obj);;PLY(*.ply);;All(*.*)");

    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        emit currentPathChanged(currentPath);

        std::cout << "loading: " << filename.toStdString() << std::endl;

        if(otherModel->load(filename.toStdString().c_str()) != 0){
            std::cerr << "Cannot open mesh file" << std::endl;
        }else{

            this->renderer->RemoveActor(canvas);
            canvas = vtkSmartPointer<vtkPropAssembly>::New();
            std::vector<Annotation*> annotations = this->mesh->getAnnotations();
            for(std::vector<Annotation*>::iterator ait = annotations.begin(); ait != annotations.end(); ait++){
                Annotation* annotation = static_cast<Annotation*>(*ait);
                Annotation* otherAnnotation;
                DrawableAnnotation* dOtherAnnotation;

                if(this->mesh->V.numels() > otherModel->V.numels() / 10)
                    otherAnnotation = annotation->transfer(otherModel, Utilities::EUCLIDEAN_DISTANCE);
                else
                    otherAnnotation = annotation->parallelTransfer(otherModel, Utilities::EUCLIDEAN_DISTANCE);

                otherAnnotation->setId(annotation->getId());
                otherAnnotation->setTag(annotation->getTag());
                otherAnnotation->setMesh(otherModel);
                otherAnnotation->setHierarchyLevel(annotation->getHierarchyLevel());

                if(otherAnnotation->getType() == AnnotationType::Surface)
                   dOtherAnnotation = new DrawableAreaAnnotation(dynamic_cast<SurfaceAnnotation*>(otherAnnotation));
                else if(otherAnnotation->getType() == AnnotationType::Line){
                   dOtherAnnotation = new DrawableLineAnnotation(dynamic_cast<LineAnnotation*>(otherAnnotation));
                   static_cast<DrawableLineAnnotation*>(dOtherAnnotation)->setLineWidth(10);
                } else if(otherAnnotation->getType() == AnnotationType::Point)
                   dOtherAnnotation = new DrawablePointAnnotation(dynamic_cast<PointAnnotation*>(otherAnnotation));

                dOtherAnnotation->setMeshPoints(mesh->getPoints());
                dOtherAnnotation->update();
                otherModel->addAnnotation(dOtherAnnotation);
            }

            otherModel->setAnnotationsModified(true);
            otherModel->update();
            QFileInfo info(filename);
            meshFilename = info.fileName();

            emit(substituteMesh(mesh, otherModel, meshFilename));
            this->setMesh(otherModel);
            this->renderer->AddActor(canvas);
            resetCamera();
        }
    }

}

void AnnotationWindow::resetCamera()
{
    vtkSmartPointer<vtkCamera> newCamera = vtkSmartPointer<vtkCamera>::New();
    newCamera->SetPosition(initialCamera->GetPosition());
    newCamera->SetFocalPoint(initialCamera->GetFocalPoint());
    newCamera->SetViewUp(initialCamera->GetViewUp());
    renderer->SetActiveCamera(newCamera);
    renderer->ResetCamera();
    renderer->GetRenderWindow()->Render();
    this->ui->qvtkWidget->update();
}

void AnnotationWindow::on_hideMeasuresButton_toggled(bool checked)
{

    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
        {
            dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->setDrawAttribute(!checked);
        }
    updateView();
}
