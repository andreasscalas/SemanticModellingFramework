#include "annotationwindow.h"
#include "ui_annotationwindow.h"
#include <QInputDialog>
#include <QListView>
#include <QMenu>
#include <QMouseEvent>
#include <geometricattribute.h>
#include <QDir>
#include <vtkIdFilter.h>
#include <QMessageBox>
#include <annotationfilemanager.h>
#include <semanticattribute.h>
#include <semanticattributedialog.h>
#include <semanticgraphinteractionstyle.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkStringArray.h>
#include <vtkDoubleArray.h>
#include <vtkRemoveIsolatedVertices.h>
#include <vtkForceDirectedLayoutStrategy.h>
#include <vtkGraphLayoutView.h>
#include <vtkEventQtSlotConnect.h>
#include <relationshipsdialog.h>


vtkStandardNewMacro(SemanticGraphInteractionStyle)

AnnotationWindow::AnnotationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AnnotationWindow)
{
    ui->setupUi(this);

    ui->caliberButton->setEnabled(false);
    ad = new AnnotationDialog(this);
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
    connect(ui->actionEditAnnotation, SIGNAL(triggered(bool)), this, SLOT(editAnnotation(bool)));
    connect(ui->rulerButton, SIGNAL(toggled(bool)), this, SLOT(rulerSelected(bool)));
    connect(ui->tapeMeasureButton, SIGNAL(toggled(bool)), this, SLOT(tapeSelected(bool)));
    connect(ui->caliberButton, SIGNAL(toggled(bool)), this, SLOT(caliperSelected(bool)));
    connect(ui->boundingButton, SIGNAL(toggled(bool)), this, SLOT(boundingSelected(bool)));
    connect(ad, SIGNAL(finalizationCalled(std::string, uchar*)), SLOT(slotFinalization(std::string, uchar*)));
//    connector = vtkSmartPointer<vtkEventQtSlotConnect>::New();
//    connector->Connect(ui->qvtkWidget->GetRenderWindow()->GetInteractor(), vtkCommand::RightButtonPressEvent, this, SLOT(openContextualMenu(vtkObject*, unsigned long, void*, void*)));
    reachedId = 0;
    isAnnotationBeingModified = false;
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetActiveCamera(vtkSmartPointer<vtkCamera>::New());
    renderer->SetBackground(1.0, 1.0, 1.0);
    renderer->AddActor(canvas);
    annotationStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    annotationStyle->setRen(renderer);
    annotationStyle->setAssembly(canvas);
    annotationStyle->setQvtkWidget(ui->qvtkWidget);
    measureStyle = vtkSmartPointer<MeasureStyle>::New();
    measureStyle->setMeshRenderer(renderer);
    measureStyle->setQvtkwidget(ui->qvtkWidget);
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
    vtkSmartPointer<vtkAreaPicker> picker = vtkSmartPointer<vtkAreaPicker>::New();
    this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(picker);
}

AnnotationWindow::~AnnotationWindow()
{
    delete ui;
}

void AnnotationWindow::updateView()
{
    mesh->draw(canvas);
    ui->qvtkWidget->update();
}

void AnnotationWindow::rulerSelected(bool isSelected)
{
    if(isSelected){
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::RULER);
    }
}

void AnnotationWindow::tapeSelected(bool isSelected)
{
    if(isSelected){
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::TAPE);
    }
}

void AnnotationWindow::caliperSelected(bool isSelected)
{
    if(isSelected){
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::CALIBER);
    }
}

void AnnotationWindow::boundingSelected(bool isSelected)
{
    if(isSelected){
        ui->actionVerticesSelection->setChecked(false);
        ui->actionLinesSelection->setChecked(false);
        ui->actionTrianglesInRectangle->setChecked(false);
        ui->actionTrianglesInPolygon->setChecked(false);
        ui->actionSelectAnnotation->setChecked(false);
        ui->actionCamera->setChecked(false);
        ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(measureStyle);
        measureStyle->setMeasureType(MeasureStyle::MeasureType::BOUNDING);
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

        static_cast<ExtendedTrimesh*>(mesh)->removeAnnotation(annotationBeingModified);
        mesh->removeAnnotation(mesh->getDAnnotations()[annotationBeingModified->getId()]);
        mesh->draw(this->canvas);
        this->canvas->Modified();
        renderer->AddActor(canvas);
        renderer->Modified();
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
    std::cout << flush;
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

    ui->qvtkWidget->update();
    this->ui->measuresList->setMesh(mesh);
    renderer->ResetCamera();
    reachedId = static_cast<unsigned int>(mesh->getAnnotations().size());

    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(mesh->getMeshSurfaceActor()->GetMapper()->GetInputAsDataSet());
    idFilter->PointIdsOn();
    idFilter->SetIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkSmartPointer<vtkIdFilter> idFilterMesh = vtkSmartPointer<vtkIdFilter>::New();
    idFilterMesh->SetInputData(mesh->getMeshPointsActor()->GetMapper()->GetInputAsDataSet());
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetIdsArrayName("OriginalMeshIds");
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

    originalCamera = vtkSmartPointer<vtkCamera>::New();
    originalCamera->DeepCopy(this->renderer->GetActiveCamera());

    for(unsigned int i = 0; i < mesh->getDAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getDAnnotations()[i]->getAttributes().size(); j++)
        {
            Attribute* a = mesh->getDAnnotations()[i]->getAttributes()[j];
            dynamic_cast<DrawableAttribute*>(a)->setDrawAttribute(false);
        }


}

void AnnotationWindow::slotSaveAnnotation()
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the annotations on the mesh",
                       ".",
                       "ANT(*.ant);;FCT(*.fct);;TRIANT(*.triant);;");

    if (!filename.isEmpty()){
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
                       "./../../",
                       "ANT(*.ant);;FCT(*.fct);;TRIANT(*.triant);;All(*.*)");

    if (!filename.isEmpty()){
        AnnotationFileManager manager;
        manager.setMesh(mesh);
        if(!manager.readAnnotations(filename.toStdString()))
            std::cout<<"Something went wrong during annotation file opening."<< std::endl<< std::flush;
        reachedId = mesh->getAnnotations().size();
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
        VTK_CREATE(vtkInteractorStyleTrackballCamera, cameraStyle);
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(cameraStyle);
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
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(verticesSelectionStyle);
        mesh->setDrawPoints(true);
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
        ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        ui->caliberButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotLinesSelection(bool checked)
{
    if(checked){
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(linesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(true);
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
        ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        ui->caliberButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotTrianglesInRectangle(bool checked)
{
    if(checked){
        trianglesSelectionStyle->setSelectionType(TriangleSelectionStyle::RECTANGLE_AREA);
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(trianglesSelectionStyle);
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
        ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        ui->caliberButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);
}

void AnnotationWindow::slotTrianglesInPolygon(bool checked)
{
    if(checked){
        trianglesSelectionStyle->setSelectionType(TriangleSelectionStyle::LASSO_AREA);
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(trianglesSelectionStyle);
        mesh->setDrawPoints(false);
        mesh->setDrawWireframe(true);
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
        ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        ui->caliberButton->setAutoExclusive(true);
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

    if(ui->qvtkWidget->GetInteractor()->GetInteractorStyle()->IsA("TriangleSelectionStyle"))
        trianglesSelectionStyle->finalizeAnnotation(id, tag, color);
    else if(ui->qvtkWidget->GetInteractor()->GetInteractorStyle()->IsA("LineSelectionStyle"))
        linesSelectionStyle->finalizeAnnotation(id, tag, color);
    else if(ui->qvtkWidget->GetInteractor()->GetInteractorStyle()->IsA("VerticesSelectionStyle"))
        verticesSelectionStyle->finalizeAnnotation(id, tag, color);
    this->ui->measuresList->update();
    this->ui->qvtkWidget->update();
}

void AnnotationWindow::slotConstrainRelationship(AnnotationsRelationship *&relationship)
{
    emit(constrainRelationship(relationship));
}

void AnnotationWindow::slotSelectAnnotation(bool checked)
{
    if(checked){
        ui->qvtkWidget->GetInteractor()->SetInteractorStyle(annotationStyle);
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
        ui->caliberButton->setAutoExclusive(false);
        ui->rulerButton->setChecked(false);
        ui->tapeMeasureButton->setChecked(false);
        ui->caliberButton->setChecked(false);
        ui->rulerButton->setAutoExclusive(true);
        ui->tapeMeasureButton->setAutoExclusive(true);
        ui->caliberButton->setAutoExclusive(true);
    } else
        ui->actionCamera->setChecked(true);

}

void AnnotationWindow::slotBuildRelationshipsGraph()
{
    GraphTemplate::Graph<Annotation*>* graph = Utilities::buildRelationshipsGraph(mesh->getAnnotations());
    mesh->setGraph(graph);
}

void AnnotationWindow::slotShowRelationships()
{

    RelationshipsDialog* d = new RelationshipsDialog(this);
    d->setMesh(mesh);
    d->update();
    d->updateView();
    d->show();
    connect(d, SIGNAL(constrainRelationship(AnnotationsRelationship*&)), this, SLOT(slotConstrainRelationship(AnnotationsRelationship*&)));

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
            measureStyle->reset();
        }
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
        if(mesh->getDAnnotations()[i]->getSelected())
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

    } else {
        QMessageBox* dialog = new QMessageBox(this);
        dialog->setWindowTitle("Error");
        dialog->setText("You need to select at least one annotation");
        dialog->show();
    }
}
void AnnotationWindow::closeEvent(QCloseEvent *event)
{
    emit(annotationWindowClosed(mesh));
}

