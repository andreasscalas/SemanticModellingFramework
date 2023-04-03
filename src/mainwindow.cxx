#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <time.h>
#include <mutex>
#include <fstream>
#include <istream>

#include <annotationwindow.h>
#include <annotationmeasuresconstraint.h>
#include <nonrigidfitting.h>
#include <shapeopmeanvaluecoordinates.h>
#include <annotationdialog.h>
#include <correspondencesdialog.h>
#include <manode.h>
#include <constraintsfilemanager.h>
#include <utilities.h>
//#include <meanvaluecoordinatesju.h>
//#include <greencoordinates.h>
//#include <drawableboundingmeasure.h>

#include <vtkOBBTree.h>
#include <vtkTetra.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCamera.h>
#include <vtkIdFilter.h>
#include <vtkPlane.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkOutlineSource.h>
#include <vtkAreaPicker.h>
#include <vtkRegularPolygonSource.h>
#include <vtkInteractorStyleImage.h>
#include <vtkProperty.h>
#include <vtkLine.h>
#include <vtkCutter.h>
//#include <vtkVectorText.h>
using namespace IMATI_STL;
using namespace std;

vtkStandardNewMacro(MeshDeformationStyle)
vtkStandardNewMacro(CageVerticesSelectionStyle)

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->init();
    this->drawInitialTetrahedron();
    ad = new AnnotationDialog(this);
    cd = new AnnotationConstraintDialog(this);
    this->ui->line->setVisible(false);
    this->ui->genericPalette->setVisible(true);
    this->lui->setVisible(true);
    this->ui->actionLayerDialog->setChecked(true);
    iren = static_cast<QVTKInteractor*>(this->ui->qvtkWidget->renderWindow()->GetInteractor());
    //Signals-Slots connection
    connect(this->ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(slotOpenMesh()));
    connect(this->ui->actionOpenCage, SIGNAL(triggered()), this, SLOT(slotOpenCage()));
    connect(this->ui->actionOpenBC, SIGNAL(triggered()), this, SLOT(slotOpenBC()));
    connect(this->ui->actionOpenConstraints, SIGNAL(triggered()), this, SLOT(slotOpenConstraints()));
    connect(this->ui->actionOpenFragment, SIGNAL(triggered()), this, SLOT(slotOpenFragment()));
    connect(this->ui->actionSaveMesh, SIGNAL(triggered()), this, SLOT(slotSaveMesh()));
    connect(this->ui->actionSaveCage, SIGNAL(triggered()), this, SLOT(slotSaveCage()));
    connect(this->ui->actionSaveBC, SIGNAL(triggered()), this, SLOT(slotSaveBC()));
    connect(this->ui->actionSaveConstraints, SIGNAL(triggered()), this, SLOT(slotSaveConstraints()));
    connect(this->ui->actionClearAll, SIGNAL(triggered()), this, SLOT(slotClearAll()));
    connect(this->ui->actionClearMesh, SIGNAL(triggered()), this, SLOT(slotClearMesh()));
    connect(this->ui->actionClearCage, SIGNAL(triggered()), this, SLOT(slotClearCage()));
    connect(this->ui->actionClearCamera, SIGNAL(triggered()), this, SLOT(slotClearCamera()));
    connect(this->ui->actionClose, SIGNAL(triggered()), this, SLOT(slotClose()));
    connect(this->ui->actionGenerate, SIGNAL(triggered()), this, SLOT(slotGenerate()));
    connect(this->ui->actionComputeCoords, SIGNAL(triggered()), this, SLOT(slotComputeCoords()));
    connect(this->ui->actionShowMesh, SIGNAL(triggered(bool)), this, SLOT(slotShowMesh(bool)));
    connect(this->ui->actionShowMeshSurface, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshSurface(bool)));
    connect(this->ui->actionShowMeshWireframe, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshWireframe(bool)));
    connect(this->ui->actionShowMeshPoints, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshPoints(bool)));
    connect(this->ui->actionShowMeshAnnotations, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshAnnotations(bool)));
    connect(this->ui->actionShowCage, SIGNAL(triggered(bool)), this, SLOT(slotShowCage(bool)));
    connect(this->ui->actionShowCageSurface, SIGNAL(triggered(bool)), this, SLOT(slotShowCageSurface(bool)));
    connect(this->ui->actionShowCageWireframe, SIGNAL(triggered(bool)), this, SLOT(slotShowCageWireframe(bool)));
    connect(this->ui->actionShowCagePoints, SIGNAL(triggered(bool)), this, SLOT(slotShowCagePoints(bool)));
    connect(this->ui->actionMeshColor, SIGNAL(triggered()), this, SLOT(slotMeshColor()));
    connect(this->ui->actionCageColor, SIGNAL(triggered()), this, SLOT(slotCageColor()));
    connect(this->ui->actionSlicerPalette, SIGNAL(triggered(bool)), this, SLOT(slotSlicerPalette(bool)));
    connect(this->ui->actionInfoPalette, SIGNAL(triggered(bool)), this, SLOT(slotInfoPalette(bool)));
    connect(this->ui->actionLayerDialog, SIGNAL(triggered(bool)), this, SLOT(slotLayerDialog(bool)));
    connect(this->ui->actionCamera, SIGNAL(triggered(bool)), this, SLOT(slotCamera(bool)));
    connect(this->ui->actionVerticesSelection, SIGNAL(triggered(bool)), this, SLOT(slotVerticesSelection(bool)));
    connect(this->ui->actionVerticesDeselection, SIGNAL(triggered(bool)), this, SLOT(slotVerticesDeselection(bool)));
    connect(this->ui->actionMeshDeformation, SIGNAL(triggered(bool)), this, SLOT(slotMeshDeformation(bool)));
    connect(this->ui->actionStretch, SIGNAL(triggered(bool)), this, SLOT(slotStretch(bool)));
    connect(this->ui->actionGreen, SIGNAL(triggered()), this, SLOT(slotGreen()));
    connect(this->ui->actionMean, SIGNAL(triggered()), this, SLOT(slotMeanValue()));
    connect(this->ui->actionVisible, SIGNAL(triggered(bool)), this, SLOT(slotVisible(bool)));
    connect(this->ui->actionSlicer, SIGNAL(triggered()), this, SLOT(slotChangeToSlicer()));
    connect(this->ui->actionCheckConstraints, SIGNAL(triggered()), this, SLOT(slotCheckConstraints()));
    connect(this->lui, SIGNAL(updateMeshView(DrawableMesh*)), this, SLOT(slotUpdateMeshView(DrawableMesh*)));
    connect(this->lui, SIGNAL(deleteMesh(DrawableMesh*)), this, SLOT(slotDeleteMesh(DrawableMesh*)));
    connect(this->lui, SIGNAL(editAnnotations(DrawableMesh*)), this, SLOT(slotEditAnnotations(DrawableMesh*)));
    connect(this->lui, SIGNAL(fitRigidly(DrawableMesh*)), this, SLOT(slotFitRigidly(DrawableMesh*)));
    connect(this->lui, SIGNAL(adaptTemplate(DrawableMesh*)), this, SLOT(slotAdaptTemplate(DrawableMesh*)));
    connect(this->sui->getXSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_xSlider_valueChanged(int)));
    connect(this->sui->getYSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_ySlider_valueChanged(int)));
    connect(this->sui->getZSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_zSlider_valueChanged(int)));
    connect(this->sui->getXSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_xSlider2_valueChanged(int)));
    connect(this->sui->getYSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_ySlider2_valueChanged(int)));
    connect(this->sui->getZSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_zSlider2_valueChanged(int)));
    connect(this->sui->getShowSkeletonCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showSkeleton_stateChanged(int)));
    connect(this->sui->getShowBoundingBoxCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showBoundingBox_stateChanged(int)));
    connect(this->sui->getShowBoundingRectangleCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showBoundingRectangle_stateChanged(int)));
    connect(this->sui->getShowDiagonalsCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showDiagonals_stateChanged(int)));
    connect(this->sui->getShowCentersCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showCenters_stateChanged(int)));
    connect(this->sui->getShowConvexHull(), SIGNAL(stateChanged(int)), this, SLOT(on_showConvexHull_stateChanged(int)));
    connect(this->sui->getShowSlicesCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showSlices_stateChanged(int)));
    connect(this->sui->getShowNodesCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showNodesBox_stateChanged(int)));
    connect(this->sui->getShowArcNodesCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showArcNodesBox_stateChanged(int)));
    connect(this->sui->getShowFittedCircleCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_showFittedCircle_stateChanged(int)));
    connect(this->sui->getClusterNumberBox(), SIGNAL(valueChanged(int)), this, SLOT(on_clusterNumberBox_valueChanged(int)));
    connect(this->sui->getMaxErrorSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_maxErrorSlider_valueChanged(int)));
    connect(this->sui->getLevelOfDetailSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_levelOfDetailSlider_valueChanged(int)));
    connect(this->sui->getClusterCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_clusterCheckBox_stateChanged(int)));
    connect(this->cd, SIGNAL(addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsConstraint(std::string, double, double, double, unsigned int, unsigned int, bool)));

}

MainWindow::~MainWindow(){
    delete ui;
    delete ad;
    delete cd;
    delete model;
    delete cage;
    delete soCoords;
    delete solver;
    delete soModelPoints;
    delete soCagePoints;
    delete slicer;
    pointsSelectionStatus.clear();
    boundingBox.clear();
    points.clear();
    clusters.clear();
    clusteringSlices.clear();
    constraints.clear();

}

void MainWindow::updateMeshView()
{
    this->ren->RemoveActor(meshAssembly);
    meshAssembly->RemovePart(this->model->getCanvas());
    meshAssembly->RemovePart(this->cage->getCanvas());
    this->model->setMeshModified(true);
    this->model->setOnlyPointsPositionsModified(true);
    this->model->update();
    this->model->draw(meshAssembly);
    this->cage->setMeshModified(true);
    this->cage->setOnlyPointsPositionsModified(true);
    this->cage->update();
    this->cage->draw(meshAssembly);
    this->ren->AddActor(meshAssembly);
    this->ui->qvtkWidget->update();
    this->meshAssembly->Modified();

}

void MainWindow::init(){

    this->currentPath = "./";
    this->newWindow = true;
    this->cageLoaded = false;
    this->showSliceOnMesh = true;
    this->showBoundingBox = false;
    this->showSkeleton = false;
    this->showNodes = true;
    this->showArcNodes = false;
    this->showFittedCircle = false;
    this->showConvexHull = false;
    this->showBoundingRectangle = false;
    this->showDiagonals = false;
    this->showCenters = false;
    this->showClusters = false;
    this->clusteringDone = false;
    this->clusterNumber = 5;
    this->reachedContraintId = 0;
    this->clusterChanged = true;
    this->constrained = false;
    this->coordsComputed = false;
    this->slicesExtracted = false;
    this->firstFragmentPositioned = false;
    this->fitTemplateOnFragments = true;
    this->sui = new SlicerPalette(this->ui->genericPalette);
    this->iui = new InfoPalette(this->ui->genericPalette);
    this->lui = new LayerDialog(this->ui->genericPalette);
    this->sui->setVisible(false);
    this->iui->setVisible(false);
    this->lui->setVisible(false);
    this->lui->setConstraintsImposed(false);
    this->sui->getLevelOfDetailSlider()->setValue(0);
    this->sui->getMaxErrorSlider()->setValue(0);
    this->levelOfDetail = MAX_LOD;
    this->allowedError = 0;
    this->translationVector << 0, 0, 0;
    this->xRotationMatrix  = Utilities::xRotationMatrix(0);
    this->yRotationMatrix = Utilities::yRotationMatrix(0);
    this->zRotationMatrix = Utilities::zRotationMatrix(0);
    this->coordType = MVC;
    this->model = nullptr;
    this->soModelPoints = nullptr;
    this->meshLoaded = false;
    this->cage = nullptr;
    this->soCagePoints = nullptr;
    this->cageLoaded = false;
    this->cagePoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelPoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->measuresAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->deformationStyle = vtkSmartPointer<MeshDeformationStyle>::New();
    this->deformationStyle->setInfoPalette(iui);
    this->soCoords = nullptr;
    this->slicer = nullptr;
    this->solver = nullptr;
    this->pointsSelectionStatus.clear();
    this->ren = vtkSmartPointer<vtkRenderer>::New();
    this->ren->SetRenderWindow(this->ui->qvtkWidget->renderWindow());
    this->ren->GetRenderWindow()->Render();
    this->ren->AddActor(meshAssembly);
    this->ren->SetBackground(1.0, 1.0, 1.0);
    this->ren->SetLayer(0);
    this->sliceRen = vtkSmartPointer<vtkRenderer>::New();
    this->sliceRen->SetBackground(1.0, 1.0, 1.0);
    this->sliceRen->GetActiveCamera()->SetPosition(center.x, center.y + 2 * totalHeight, center.z);
    this->sliceRen->GetActiveCamera()->SetViewUp(0, 0, 1);
    this->sliceRen->GetActiveCamera()->SetFocalPoint(center.x, center.y, center.z);
    this->ui->qvtkWidget->renderWindow()->AddRenderer(ren);
    this->sui->getSlicerWidget()->renderWindow()->AddRenderer(sliceRen);
    this->sui->getYSlider()->setValue(50);
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->ui->actionVisible->setChecked(true);
    this->ui->actionCamera->setChecked(true);
    this->ui->actionVerticesSelection->setChecked(false);
    this->ui->actionVerticesDeselection->setChecked(false);
    this->ui->actionMeshDeformation->setChecked(false);
    this->ui->actionStretch->setChecked(false);
    this->ui->actionGreen->setChecked(false);
    this->ui->actionMean->setChecked(true);
    this->ui->actionDelete->setChecked(false);
    this->ui->actionRectSelection->setChecked(false);
    this->ui->actionLassoSelection->setChecked(false);
    this->ui->actionAnnotation->setChecked(false);
    this->ui->actionCamera->setEnabled(false);
    this->ui->actionVerticesSelection->setEnabled(false);
    this->ui->actionVerticesDeselection->setEnabled(false);
    this->ui->actionMeshDeformation->setEnabled(false);
    this->ui->actionStretch->setEnabled(false);
    this->ui->actionGreen->setEnabled(false);
    this->ui->actionMean->setEnabled(false);
    this->ui->actionDelete->setEnabled(false);
    this->ui->actionLineSelection->setEnabled(false);
    this->ui->actionRectSelection->setEnabled(false);
    this->ui->actionLassoSelection->setEnabled(false);
    this->ui->actionAnnotation->setEnabled(false);
    this->ui->actionShowMesh->setEnabled(false);
    this->ui->actionShowCage->setEnabled(false);
    this->ui->actionShowMesh->setChecked(false);
    this->ui->actionShowCage->setChecked(false);
    this->ui->actionSlicerPalette->setEnabled(false);
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setEnabled(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->sui->getXSlider()->setEnabled(true);
    this->sui->getYSlider()->setEnabled(true);
    this->sui->getZSlider()->setEnabled(true);
    this->sui->getXSlider2()->setEnabled(true);
    this->sui->getYSlider2()->setEnabled(true);
    this->sui->getZSlider2()->setEnabled(true);
    this->sui->getMaxErrorSlider()->setEnabled(true);
    this->sui->getLevelOfDetailSlider()->setEnabled(true);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    this->initialCamera = vtkSmartPointer<vtkCamera>::New();
    this->initialCamera->DeepCopy(ren->GetActiveCamera());

}

void MainWindow::write(std::string message){

//    vtkSmartPointer<vtkVectorText> text = vtkSmartPointer<vtkVectorText>::New();
//    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    text->SetText(message.c_str());
//    mapper->SetInputConnection(text->GetOutputPort());
//    actor->SetMapper(mapper);
//    actor->GetProperty()->SetColor(0,0,0);
//    this->meshAssembly->AddPart(actor);
//    this->ren->ResetCamera();
//    this->ui->qvtkWidget->update();

}

void MainWindow::drawInitialTetrahedron()
{
    vtkSmartPointer< vtkPoints > points = vtkSmartPointer< vtkPoints > :: New();
    points->InsertNextPoint(-0.32, -0.75, 0);
    points->InsertNextPoint(1, 0, -0.5);
    points->InsertNextPoint(-0.32, 0.75, 0);
    points->InsertNextPoint(-0.32, 0, -1.52);

    vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid1 = vtkSmartPointer<vtkUnstructuredGrid>::New();
     unstructuredGrid1->SetPoints(points);

     vtkIdType ptIds[] = {0, 1, 2, 3};
     unstructuredGrid1->InsertNextCell( VTK_TETRA, 4, ptIds );

    vtkSmartPointer<vtkTetra> tetra = vtkSmartPointer<vtkTetra>::New();
    tetra->GetPointIds()->SetId(0, 0);
    tetra->GetPointIds()->SetId(1, 1);
    tetra->GetPointIds()->SetId(2, 2);
    tetra->GetPointIds()->SetId(3, 3);
    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
    cellArray->InsertNextCell(tetra);
    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstructuredGrid1);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.752941176,0.752941176,0.752941176);
    this->meshAssembly->AddPart(actor);
    this->ren->ResetCamera();
    this->ui->qvtkWidget->update();
}

void MainWindow::updateView(){

    vtkSmartPointer<vtkPlane> intersectingPlane = vtkSmartPointer<vtkPlane>::New();
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    intersectingPlane->SetOrigin(center.x, center.y, center.z);
    intersectingPlane->SetNormal(normal.x, normal.y, normal.z);
    this->ren->Clear();
    this->sliceRen->Clear();

    if(model != nullptr)
    {

        for(unsigned int i = 0; i < model->getAnnotations().size(); i++)
            for(unsigned int j = 0; j < model->getAnnotations()[i]->getAttributes().size(); j++)
                dynamic_cast<DrawableAttribute*>(model->getAnnotations()[i]->getAttributes()[j])->setRenderer(ren);
        this->model->draw(meshAssembly);
        cutter->SetCutFunction(intersectingPlane);
        poly->SetPoints(this->model->getPoints());
        poly->SetPolys(this->model->getTriangles());
        cutter->SetInputData(poly);
        cutter->Update();
        vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
        pd = cutter->GetOutput();
        vtkSmartPointer<vtkPolyDataMapper> cutterMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        cutterMapper->SetInputData(pd);

        vtkSmartPointer<vtkActor> sliceActor = vtkSmartPointer<vtkActor>::New();
        sliceActor->SetMapper(cutterMapper);
        sliceActor->GetProperty()->SetOpacity(1);
        sliceActor->GetProperty()->SetLineWidth(3);
        sliceActor->GetProperty()->SetColor(0, 0.5, 0);
        sliceActor->GetProperty()->SetRepresentationToWireframe();
        if(this->showSliceOnMesh && this->sui->isVisible())
            this->visualPropertiesAssembly->AddPart(sliceActor);
    }
    if(cage != nullptr)
        this->cage->draw(meshAssembly);
    this->meshAssembly->RemovePart(visualPropertiesAssembly);
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->sliceRen->RemoveActor(sliceAssembly);
    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::New();

    for(unsigned int i = 0; i < fragments.size(); i++)
        for(unsigned int j = 0; j < fragments[i]->getAnnotations().size(); j++)
            for(unsigned int k = 0; k < fragments[i]->getAnnotations()[j]->getAttributes().size(); k++)
                dynamic_cast<DrawableAttribute*>(fragments[i]->getAnnotations()[j]->getAttributes()[k])->setRenderer(ren);
    if(this->showBoundingBox){

        if(this->boundingBox.size() == 0){
            vector<double> bb = Utilities::getOBB(model);
            this->boundingBox.insert(this->boundingBox.end(), bb.begin(), bb.end());
        }

        vtkSmartPointer<vtkPolyData> boundingBoxData = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyDataMapper> boundingBoxMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> boundingBoxActor = vtkSmartPointer<vtkActor>::New();
        vtkSmartPointer<vtkOutlineSource> boundingBoxSource = vtkSmartPointer<vtkOutlineSource>::New();

        boundingBoxSource->SetBoxTypeToOriented();
        boundingBoxSource->SetCorners(this->boundingBox.data());
        boundingBoxSource->Update();
        boundingBoxMapper->SetInputConnection(boundingBoxSource->GetOutputPort());
        boundingBoxActor->SetMapper(boundingBoxMapper);
        boundingBoxActor->GetProperty()->SetColor(1, 0, 0);
        this->visualPropertiesAssembly->AddPart(boundingBoxActor);
    }

    if(this->showClusters){

        unsigned int clusterPosition = 0;
        for(vector<vector<unsigned int> >::iterator cit = this->clusters.begin(); cit != this->clusters.end(); cit++){
            vector<unsigned int> cluster = static_cast<vector<unsigned int> >(*cit);
            for(vector<unsigned int>::iterator pit = cluster.begin(); pit != cluster.end(); pit++){

                Slice* s = this->clusteringSlices[*pit];
                vector<IMATI_STL::Point*> boundary = s->getBoundary();
                vtkSmartPointer<vtkActor> sliceBoundaryActor = vtkSmartPointer<vtkActor>::New();
                Utilities::renderSpline(boundary, sliceBoundaryActor);
                vector<double> color;
                for(unsigned int i = 0; i < s->getHoles().size(); i++){
                    color.clear();
                    vtkSmartPointer<vtkActor> sliceHoleActor = vtkSmartPointer<vtkActor>::New();
                    Utilities::renderSpline(s->getHoles()[i], sliceHoleActor);
                    color.insert(color.end(), colors[clusterPosition].begin(), colors[clusterPosition].end());
                    sliceHoleActor->GetProperty()->SetColor(color.data());
                    sliceHoleActor->GetProperty()->SetLineWidth(2);
                    this->visualPropertiesAssembly->AddPart(sliceHoleActor);
                }
                color.clear();
                color.insert(color.end(), colors[clusterPosition].begin(), colors[clusterPosition].end());
                sliceBoundaryActor->GetProperty()->SetColor(color.data());
                sliceBoundaryActor->GetProperty()->SetLineWidth(2);
                this->visualPropertiesAssembly->AddPart(sliceBoundaryActor);
            }
            clusterPosition++;
        }
    }

    if(this->slicesExtracted){
        vector<Slice*> slices = this->slicer->getSlices();

        IMATI_STL::Point slicesCenter(0, 0, 0);
        for(unsigned int i = 0; i < slices.size(); i++){

            DrawableMesh* sliceMesh = new DrawableMesh(static_cast<DrawableMesh*>(slices[i]->getSlice()));
            sliceMesh->buildVTKStructure();
            sliceMesh->setMeshModified(true);
            sliceMesh->setAnnotationsModified(true);
            sliceMesh->update();
            slicesCenter += sliceMesh->getCenter();
            if(this->sui->isVisible()){
                sliceMesh->draw(this->sliceAssembly);
                vector<IMATI_STL::Point*> contour = slices[i]->getBoundary();
                Utilities::removeCreeks(contour, 10);

                Eigen::Matrix4d transformation = zRotationMatrix * (yRotationMatrix * (xRotationMatrix * translationMatrix));

                if(contour.size() > 2){
                    vector<IMATI_STL::Point*> boundary = Utilities::transformVertexList(contour, transformation);
                    vtkSmartPointer<vtkActor> sliceContour2DActor = vtkSmartPointer<vtkActor>::New();
                    Utilities::renderSpline(contour, sliceContour2DActor);
                    sliceContour2DActor->GetProperty()->SetColor(0, 0.5, 0);
                    sliceContour2DActor->GetProperty()->SetLineWidth(2);

                    vtkSmartPointer<vtkActor> sliceContour3DActor = vtkSmartPointer<vtkActor>::New();
                    Utilities::renderSpline(boundary, sliceContour3DActor);
                    sliceContour3DActor->GetProperty()->SetColor(0, 0.5, 0);
                    sliceContour3DActor->GetProperty()->SetLineWidth(2);
                    sliceAssembly->AddPart(sliceContour2DActor);
                    visualPropertiesAssembly->AddPart(sliceContour3DActor);
                    for(unsigned int j = 0; j < slices[i]->getHoles().size(); j++){
                        vector<IMATI_STL::Point*> hole = Utilities::transformVertexList(slices[i]->getHoles()[j], slicer->getTransformationMatrix());
                        vtkSmartPointer<vtkActor> sliceHoleContour2DActor = vtkSmartPointer<vtkActor>::New();
                        Utilities::renderSpline(slices[i]->getHoles()[j], sliceHoleContour2DActor);
                        sliceHoleContour2DActor->GetProperty()->SetColor(0, 0.5, 0);
                        sliceHoleContour2DActor->GetProperty()->SetLineWidth(2);
                        sliceAssembly->AddPart(sliceHoleContour2DActor);
                        vtkSmartPointer<vtkActor> sliceHoleContour3DActor = vtkSmartPointer<vtkActor>::New();
                        Utilities::renderSpline(hole, sliceHoleContour3DActor);
                        sliceHoleContour3DActor->GetProperty()->SetColor(0, 0.5, 0);
                        sliceHoleContour3DActor->GetProperty()->SetLineWidth(2);
                        visualPropertiesAssembly->AddPart(sliceHoleContour3DActor);
                    }
                }
            }

            if(showFittedCircle){
                vector<pair<IMATI_STL::Point*, double> > circles = slices[i]->getBestFittedCircles();

                for(unsigned int j = 0; j < circles.size(); j++){
                    vtkSmartPointer<vtkRegularPolygonSource> circleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
                    circleSource->GeneratePolygonOff();
                    circleSource->SetNumberOfSides(50);
                    circleSource->SetNormal(0, 1, 0);
                    circleSource->SetRadius(circles[j].second);
                    circleSource->SetCenter(circles[j].first->x, circles[j].first->y, circles[j].first->z);
                    vtkSmartPointer<vtkPolyDataMapper> circleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    circleMapper->SetInputConnection(circleSource->GetOutputPort());
                    vtkSmartPointer<vtkActor> circleActor = vtkSmartPointer<vtkActor>::New();
                    circleActor->SetMapper(circleMapper);
                    vector<double> color = {BORDEAUX[0], BORDEAUX[1], BORDEAUX[2]};
                    circleActor->GetProperty()->SetColor(color.data());
                    circleActor->GetProperty()->SetLineWidth(3);
                    sliceAssembly->AddPart(circleActor);
                }

            }

            if(showCenters){

                vtkSmartPointer<vtkPolyData> tmpData = vtkSmartPointer<vtkPolyData>::New();
                vtkSmartPointer<vtkPolyData> centersData = vtkSmartPointer<vtkPolyData>::New();
                vtkSmartPointer<vtkPoints> centersPoints = vtkSmartPointer<vtkPoints>::New();
                vtkSmartPointer<vtkVertexGlyphFilter> centersFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
                IMATI_STL::Point c = slices[i]->getSlice()->getCenter();
                if(showFittedCircle){
                    vector<pair<IMATI_STL::Point*, double> > circles = slices[i]->getBestFittedCircles();
                    pair<IMATI_STL::Point*, double> circle = circles[0];
                    centersPoints->InsertNextPoint(circle.first->x, circle.first->y, circle.first->z);
                }
                centersPoints->InsertNextPoint(c.x, c.y, c.z);
                tmpData->SetPoints(centersPoints);
                centersFilter->SetInputData(tmpData);
                centersFilter->Update();
                centersData->ShallowCopy(centersFilter->GetOutput());
                vtkSmartPointer<vtkPolyDataMapper> centersMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                centersMapper->SetInputData(centersData);
                vtkSmartPointer<vtkActor> centersActor = vtkSmartPointer<vtkActor>::New();
                centersActor->SetMapper(centersMapper);
                centersActor->GetProperty()->SetPointSize(3);
                centersActor->GetProperty()->SetColor(1, 0, 0);
                sliceAssembly->AddPart(centersActor);

            }

            if(showConvexHull){

                slices[i]->computeConvexHull();
                vector<IMATI_STL::Point*> convexHull = slices[i]->getConvexHull();
                vtkSmartPointer<vtkActor> chActor = vtkSmartPointer<vtkActor>::New();
                Utilities::renderSpline(convexHull, chActor);
                chActor->GetProperty()->SetColor(1,0,1);
                sliceAssembly->AddPart(chActor);

            }

            if(showBoundingRectangle){

                slices[i]->computeBoundingBox();
                vector<IMATI_STL::Point*> bb = slices[i]->getBoundingBox();
                bb.push_back(bb[0]);
                vtkSmartPointer<vtkActor> boundingRectangleActor = vtkSmartPointer<vtkActor>::New();
                Utilities::renderSpline(bb, boundingRectangleActor);
                boundingRectangleActor->GetProperty()->SetColor(1, 0, 1);
                sliceAssembly->AddPart(boundingRectangleActor);

            }

            if(showDiagonals){

                slices[i]->computeBoundingBox();
                vtkSmartPointer<vtkPolyData> diagonalsData = vtkSmartPointer<vtkPolyData>::New();
                vtkSmartPointer<vtkPoints> diagonalsPoints = vtkSmartPointer<vtkPoints>::New();
                vtkSmartPointer<vtkCellArray> diagonalsLines = vtkSmartPointer<vtkCellArray>::New();

                pair<IMATI_STL::Point, IMATI_STL::Point> minDiagonal = slices[i]->getMinDiagonal();
                pair<IMATI_STL::Point, IMATI_STL::Point> maxDiagonal = slices[i]->getMaxDiagonal();

                diagonalsPoints->InsertNextPoint(minDiagonal.first.x, minDiagonal.first.y, minDiagonal.first.z);
                diagonalsPoints->InsertNextPoint(minDiagonal.second.x, minDiagonal.second.y, minDiagonal.second.z);
                diagonalsPoints->InsertNextPoint(maxDiagonal.first.x, maxDiagonal.first.y, maxDiagonal.first.z);
                diagonalsPoints->InsertNextPoint(maxDiagonal.second.x, maxDiagonal.second.y, maxDiagonal.second.z);
                vtkSmartPointer<vtkLine> diagonal1 = vtkSmartPointer<vtkLine>::New();
                diagonal1->GetPointIds()->SetId(0, 0);
                diagonal1->GetPointIds()->SetId(1, 1);
                diagonalsLines->InsertNextCell(diagonal1);
                vtkSmartPointer<vtkLine> diagonal2 = vtkSmartPointer<vtkLine>::New();
                diagonal2->GetPointIds()->SetId(0, 2);
                diagonal2->GetPointIds()->SetId(1, 3);
                diagonalsLines->InsertNextCell(diagonal2);


                diagonalsData->SetPoints(diagonalsPoints);
                diagonalsData->SetLines(diagonalsLines);

                vtkSmartPointer<vtkPolyDataMapper> diagonalsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                diagonalsMapper->SetInputData(diagonalsData);
                vtkSmartPointer<vtkActor> diagonalsActor = vtkSmartPointer<vtkActor>::New();
                diagonalsActor->SetMapper(diagonalsMapper);
                diagonalsActor->GetProperty()->SetColor(1,0,1);
                sliceAssembly->AddPart(diagonalsActor);

            }

            if(showSkeleton){

                AndreasStructures::MANode* skeleton = new AndreasStructures::MANode(slices[i]->getSkeleton());

                map<Vertex*, double> pointsCurvature = Utilities::computePolygonCurvature(slices[i]->getSlice());
                skeleton = Utilities::simplifySkeleton(skeleton, levelOfDetail);

                vector<AndreasStructures::MAArcPath*> arcs = skeleton->getGraphArcs();
                vtkSmartPointer<vtkPolyData> skeletonData = vtkSmartPointer<vtkPolyData>::New();
                vtkSmartPointer<vtkPolyData> skeletonNodesData = vtkSmartPointer<vtkPolyData>::New();

                vtkSmartPointer<vtkPoints> skeletonPoints = vtkSmartPointer<vtkPoints>::New();
                vtkSmartPointer<vtkPoints> skeletonNodesPoints = vtkSmartPointer<vtkPoints>::New();
                vtkSmartPointer<vtkCellArray> skeletonLines = vtkSmartPointer<vtkCellArray>::New();

                vtkSmartPointer<vtkIdList> cellpoints = vtkIdList::New();
                for(vector<AndreasStructures::MAArcPath*>::iterator ait = arcs.begin(); ait != arcs.end(); ait++){
                    AndreasStructures::MAArcPath* p = static_cast<AndreasStructures::MAArcPath*>(*ait);
                    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                    Vertex* p1 = p->getN1()->getPoint();
                    Vertex* p2 = p->getN2()->getPoint();
                    vtkIdType pid1, pid2;

                    pid1 = skeletonPoints->InsertNextPoint(p1->x, p1->y, p1->z);
                    cellpoints->InsertNextId(pid1);

                    vector<IMATI_STL::Point*> commonPath = p->getPath();

                    for(vector<IMATI_STL::Point*>::iterator pit = commonPath.begin(); pit != commonPath.end(); pit++){
                        pid2 = skeletonPoints->InsertNextPoint((*pit)->x, (*pit)->y, (*pit)->z);
                        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                        line->GetPointIds()->SetId(0, pid1);
                        line->GetPointIds()->SetId(1, pid2);
                        skeletonLines->InsertNextCell(line);
                        pid1 = pid2;
                    }

                    pid2 = skeletonPoints->InsertNextPoint(p2->x, p2->y, p2->z);
                    cellpoints->InsertNextId(pid2);


                    vtkSmartPointer<vtkLine> lastLine = vtkSmartPointer<vtkLine>::New();
                    line->GetPointIds()->SetId(0, pid1);
                    line->GetPointIds()->SetId(1, pid2);
                    skeletonLines->InsertNextCell(line);
                }

                for(int i = 0; i < cellpoints->GetNumberOfIds(); i++){
                    vtkIdType id = cellpoints->GetId(i);
                    skeletonNodesPoints->InsertNextPoint(skeletonPoints->GetPoint(id));
                }
                skeletonNodesData->SetPoints(skeletonNodesPoints);
                vtkSmartPointer<vtkVertexGlyphFilter> filter1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
                filter1->SetInputData(skeletonNodesData);
                filter1->Update();
                vtkSmartPointer<vtkVertexGlyphFilter> filter2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
                filter2->SetInputData(skeletonData);
                filter2->Update();

                skeletonData->SetPoints(skeletonPoints);
                skeletonData->SetLines(skeletonLines);
                vtkSmartPointer<vtkPolyDataMapper> skeletonMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                vtkSmartPointer<vtkActor> skeletonActor = vtkSmartPointer<vtkActor>::New();
                skeletonMapper->SetInputData(skeletonData);
                skeletonActor->SetMapper(skeletonMapper);
                skeletonActor->GetProperty()->SetLineWidth(2);
                sliceAssembly->AddPart(skeletonActor);
                skeletonActor->GetProperty()->SetColor(0, 0, 0);

                if(showArcNodes){
                    vtkSmartPointer<vtkPolyDataMapper> skeletonPointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    vtkSmartPointer<vtkActor> skeletonPointsActor = vtkSmartPointer<vtkActor>::New();
                    skeletonPointsMapper->SetInputConnection(filter2->GetOutputPort());
                    skeletonPointsActor->SetMapper(skeletonPointsMapper);
                    skeletonPointsActor->GetProperty()->SetColor(255, 0, 0);
                    skeletonPointsActor->GetProperty()->SetRenderPointsAsSpheres(true);
                    skeletonPointsActor->GetProperty()->SetPointSize(5);
                    sliceAssembly->AddPart(skeletonPointsActor);
                }

                if(showNodes){
                    vtkSmartPointer<vtkPolyDataMapper> skeletonNodesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    vtkSmartPointer<vtkActor> skeletonNodesActor = vtkSmartPointer<vtkActor>::New();
                    skeletonNodesMapper->SetInputConnection(filter1->GetOutputPort());
                    skeletonNodesActor->SetMapper(skeletonNodesMapper);
                    skeletonNodesActor->GetProperty()->SetColor(0, 0, 255);
                    skeletonNodesActor->GetProperty()->SetPointSize(5);
                    skeletonNodesActor->GetProperty()->SetRenderPointsAsSpheres(true);
                    sliceAssembly->AddPart(skeletonNodesActor);
                }
            }

            //delete(sliceMesh);
        }

        slicesCenter /= slices.size();
        visualPropertiesAssembly->Modified();
        meshAssembly->AddPart(visualPropertiesAssembly);
        sliceRen->AddActor(sliceAssembly);
        double f[3] = {slicesCenter.x, slicesCenter.y, slicesCenter.z};
        sliceRen->GetActiveCamera()->SetFocalPoint(f);
        sliceRen->GetActiveCamera()->SetPosition(f[0], f[1] + 100, f[2]);
        sliceRen->Modified();
        this->sui->getSlicerWidget()->update();
    }

    this->ui->qvtkWidget->update();

}

void MainWindow::updateIntersection(){

    if(meshLoaded == true){
        Eigen::Vector4d omogeneousNormal = {0, 1, 0, 1};
        Eigen::Vector4d omogeneousCenter = {0, 0, 0, 1};
        Eigen::Vector4d omogeneousCenterResult = translationMatrix * omogeneousCenter;
        Eigen::Vector4d omogeneousNormalResult = zRotationMatrix * (yRotationMatrix * (xRotationMatrix * omogeneousNormal));
        Eigen::Matrix4d transformationMatrix = zRotationMatrix * (yRotationMatrix * (xRotationMatrix * translationMatrix));
        this->center.setValue(omogeneousCenterResult(0), omogeneousCenterResult(1), omogeneousCenterResult(2));
        this->normal.setValue(omogeneousNormalResult(0), omogeneousNormalResult(1), omogeneousNormalResult(2));
        if(slicer != nullptr)
            delete slicer;
        slicer = new Slicer();
        slicer->setCenter(center);
        slicer->setNormal(normal);
        slicer->setMesh(model);
        slicer->setMaxSimplificationError(allowedError);
        slicer->setTransformationMatrix(transformationMatrix);
        slicer->slice();

        vector<Slice*> slices = slicer->getSlices();
        for(unsigned int i = 0; i < static_cast<unsigned int>(slices.size()); i++){
            AndreasStructures::MANode* skeleton = Utilities::medialAxisTransform(slices[i]->getSlice());
            slices[i]->setSkeleton(skeleton);
            slices[i]->printInformation();
        }
        slicer->setSlices(slices);

        this->slicesExtracted = true;

        updateView();
        this->ui->qvtkWidget->update();
    }

}

void MainWindow::clear(){

    this->setWindowTitle("MainWindow");
    this->meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->newWindow = true;
    this->cageLoaded = false;
    this->showClusters = false;
    this->clusteringDone = false;
    this->slicesExtracted = false;
    this->firstFragmentPositioned = false;
    if(model != nullptr)
        delete model;
    this->model = nullptr;
    if(soModelPoints != nullptr)
        delete  soModelPoints;
    soModelPoints = nullptr;
    this->model = new DrawableMesh();
    this->meshLoaded = false;
    this->lui->removeCage(cage);
    if(cage != nullptr)
        delete cage;
    if(soCagePoints != nullptr)
        delete  soCagePoints;
    soCagePoints = nullptr;
    if(this->cage != nullptr)
        delete this->cage;
    this->cage = nullptr;
    this->cagePoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelPoints = vtkSmartPointer<vtkPolyData>::New();
    this->pointsSelectionStatus.clear();
    if(soCoords != nullptr)
        delete soCoords;
    if(slicer != nullptr)
        delete slicer;
    if(solver != nullptr)
        delete solver;
    this->constraints.clear();
    this->points.clear();
    this->clusters.clear();
    this->clusteringSlices.clear();
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->sui->getSlicerWidget()->renderWindow()->RemoveRenderer(sliceRen);
    meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    sliceAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    sliceRen = vtkSmartPointer<vtkRenderer>::New();
    sliceRen->AddActor(sliceAssembly);
    sliceRen->SetBackground(1.0,1.0,1.0);
    this->resetCamera();
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->sui->setVisible(false);
    this->sui->getSlicerWidget()->renderWindow()->AddRenderer(sliceRen);
    this->sui->getClusterCheckBox()->setChecked(false);
    this->ui->actionVerticesSelection->setEnabled(false);
    this->ui->actionVerticesDeselection->setEnabled(false);
    this->ui->actionMeshDeformation->setEnabled(false);
    this->ui->actionStretch->setEnabled(false);
    this->sui->getTabWidget()->setEnabled(false);
    this->ui->actionAddVerticesRectangle->setEnabled(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    vtkSmartPointer<vtkInteractorStyleImage> imageStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    this->sui->getSlicerWidget()->renderWindow()->GetInteractor()->SetInteractorStyle(imageStyle);
    //this->clearAll();

}

void MainWindow::slotOpenFile(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();

        if(model->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{
            this->model->draw(meshAssembly);
            this->meshLoadedPreparation();
        }
    }

}

void MainWindow::slotOpenMesh(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        QFileInfo info(filename);
        modelFilename = info.fileName();
        currentPath = info.absolutePath().toStdString();
        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();
        this->model = new DrawableMesh();

        if(this->model->load(filename.toStdString().c_str()) != 0)
            this->write("Can't open mesh file");
        else
            this->meshLoadedPreparation();

        this->initialCamera = vtkSmartPointer<vtkCamera>::New();
        this->initialCamera->DeepCopy(ren->GetActiveCamera());

    }

}

void MainWindow::slotOpenCage(){

    if(!cageLoaded){

        QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D cage",
                           QString::fromStdString(currentPath),
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

        if (!filename.isEmpty()){

            QFileInfo info(filename);
            cageFilename = info.fileName();
            currentPath = info.absolutePath().toStdString();
            std::cout << "loading: " << filename.toStdString() << std::endl << std::flush;
            this->cage = new DrawableMesh();
            if(cage->load(filename.toStdString().c_str()) != 0)
                this->write("Can't open cage file");

            else{

                this->lui->addCage(cageFilename, cage);
                this->soCagePoints = new ShapeOp::MatrixX3();
                soCagePoints->resize(cage->V.numels(), 3);
                unsigned int i = 0;
                for(IMATI_STL::Node* n = cage->V.head(); n != nullptr; n = n->next()){
                    Vertex* v = static_cast<Vertex*>(n->data);
                    ShapeOp::Vector3 p = {v->x, v->y, v->z};
                    soCagePoints->row(i++) = p;
                }

                this->cageLoaded = true;
                this->cage->setIsCage(true);
                this->cage->setDrawSurface(false);
                this->cage->setDrawWireframe(true);
                this->cage->setDrawPoints(true);
                this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
                this->cagePoints->SetPoints(this->cage->getPoints());
                vtkSmartPointer<vtkIdFilter> idFilterCage = vtkSmartPointer<vtkIdFilter>::New();
                idFilterCage->SetInputData(cagePoints);
                idFilterCage->PointIdsOn();
                idFilterCage->SetPointIdsArrayName("OriginalIds");
                idFilterCage->Update();
                vtkPolyData* inputCage = static_cast<vtkPolyData*>(idFilterCage->GetOutput());

                this->selectionStyle->SetCurrentRenderer(ren);
                this->selectionStyle->setPoints(inputCage);
                this->selectionStyle->setAssembly(this->meshAssembly);
                this->selectionStyle->setModel(this->model);
                this->selectionStyle->setCage(this->cage);
                this->selectionStyle->setAnnotationToCage(&(this->annotationToCage));
                this->selectionStyle->setSelectedPoints(&(this->pointsSelectionStatus));
                this->selectionStyle->setQvtkwidget(this->ui->qvtkWidget);
                this->selectionStyle->resetSelection();

                this->deformationStyle->setData(inputCage);
                this->deformationStyle->setModel(this->model);
                this->deformationStyle->setModelPoints(this->soModelPoints);
                this->deformationStyle->setCage(this->cage);
                this->deformationStyle->setCagePoints(this->soCagePoints);
                this->deformationStyle->setAssembly(this->meshAssembly);
                this->deformationStyle->setSelectedPoints(&(this->pointsSelectionStatus));
                this->deformationStyle->setCoordsComputed(false);
                this->deformationStyle->setQvtkwidget(this->ui->qvtkWidget);

                this->ui->actionVerticesSelection->setEnabled(true);
                this->ui->actionVerticesDeselection->setEnabled(true);
                this->ui->actionMeshDeformation->setEnabled(true);
                this->ui->actionStretch->setEnabled(true);
                this->ui->actionMean->setEnabled(true);
                this->ui->actionShowCage->setEnabled(true);
                this->ui->actionShowCage->setChecked(true);
                this->ui->cageVerticesLabel->setText(this->ui->cageVerticesLabel->text() + QString::number(cage->V.numels()));
                this->ui->cageEdgesLabel->setText(this->ui->cageEdgesLabel->text() + QString::number(cage->E.numels()));
                this->ui->cageTrianglesLabel->setText(this->ui->cageTrianglesLabel->text() + QString::number(cage->T.numels()));
            }

            cage->draw(meshAssembly);
            this->ui->qvtkWidget->update();

        }

    }


}

void MainWindow::slotOpenBC(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Choose a BC file",
                       QString::fromStdString(currentPath),
                       "Coords(*.coord);; All(*.*)");

    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        ifstream loadfile;
        loadfile.open(filename.toStdString());
        std::string line;
        std::getline(loadfile, line);
        loadfile.close();
        this->soCoords = new ShapeOpMeanValueCoordinates(model, cage, soModelPoints, soCagePoints);
        this->soCoords->loadCoordinates(filename.toStdString());
        this->modelToCage.clear();
        for(IMATI_STL::Node* n = this->model->V.head(); n != nullptr; n = n->next())
        {
            Vertex* v = static_cast<Vertex*>(n->data);
            this->modelToCage[this->model->getPointId(v)] = this->soCoords->getMaxInfluenceCageVertices(this->model->getPointId(v));
        }
        this->deformationStyle->setSoCoords(this->soCoords);
        this->deformationStyle->setCoordsComputed(true);
        this->coordsComputed = true;
        this->lui->setBcComputed(true);
    }
}

void MainWindow::slotOpenConstraints()
{
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Load the constraints on the model",
                       QString::fromStdString(currentPath),
                       "CSTR(*.cstr);;");


    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        ConstraintsFileManager manager;
        manager.setMesh(model);
        manager.setSoPoints(this->soModelPoints);
        if(!manager.readConstraints(filename.toStdString()))
            std::cout<<"Something went wrong during annotation file opening."<< std::endl<< std::flush;
        this->constraints = manager.getConstraints();
        model->update();
        model->draw(meshAssembly);
    }
}

void MainWindow::slotOpenFragment()
{
    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        DrawableMesh* newFragment = new DrawableMesh();

        if(newFragment->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{
            QFileInfo info(filename);
            currentPath = info.absolutePath().toStdString();
            newFragment->setIsTemplate(false);
            newFragment->setIsCage(false);
            fragments.push_back(newFragment);
            fragmentsNames.push_back(info.fileName());
            this->lui->addFragment(info.fileName(), newFragment);
            newFragment->setDrawWireframe(false);
            ShapeOp::MatrixX3* newFragmentPoints = new ShapeOp::MatrixX3();
            newFragmentPoints->resize(newFragment->V.numels(), 3);
            unsigned int i = 0;
            for(IMATI_STL::Node* n = newFragment->V.head(); n != nullptr; n = n->next()){
                Vertex* v = static_cast<Vertex*>(n->data);
                ShapeOp::Vector3 p = {v->x, v->y, v->z};
                newFragmentPoints->row(i++) = p;
            }
            soFragmentsPoints.push_back(newFragmentPoints);
            meshAssembly->RemovePart(visualPropertiesAssembly);
            visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
            newFragment->setMeshModified(true);
            newFragment->setAnnotationsModified(true);
            newFragment->update();
            newFragment->draw(meshAssembly);
            meshAssembly->AddPart(visualPropertiesAssembly);
            ren->Modified();
            this->ui->qvtkWidget->update();
        }
    }
}

void MainWindow::slotSaveFile(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the deformed model",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty()){

        string fname = filename.toStdString();

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        model->save(filename.toStdString().c_str());
        int pointPos = filename.lastIndexOf('.');
        filename.insert(pointPos, "_cage");
        cage->save(filename.toStdString().c_str());
        filename.truncate(pointPos);
        filename.append(".coord");
        soCoords->saveCoordinates(fname.substr(fname.find_last_of('/')));

    }

}

void MainWindow::slotSaveMesh(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the mesh",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        model->save(filename.toStdString().c_str(),0);
    }

}

void MainWindow::slotSaveCage(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the cage",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        cage->save(filename.toStdString().c_str(),0);
    }

}

void MainWindow::slotSaveBC(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the Barycentric Coordinates",
                       ".",
                       "COORD(*.coord);;");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        soCoords->saveCoordinates(filename.toStdString().c_str());
    }
}

void MainWindow::slotSaveConstraints()
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the constraints on the mesh",
                       ".",
                       "CSTR(*.cstr);;");

    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        ConstraintsFileManager manager;
        manager.setMesh(model);
        manager.setConstraints(this->constraints);
        if(!manager.writeConstraints(filename.toStdString()))
            std::cout << "Something went wrong during constraints file writing." << std::endl << std::flush;

    }
}

void MainWindow::slotClearMesh(){

    lui->removeMainMesh(model);
    clearMesh();

}

void MainWindow::slotClearCage(){
    lui->removeCage(cage);
    clearCage();
}

void MainWindow::slotClearConstraints()
{
    clearConstraints();
}

void MainWindow::slotClearCamera()
{
    resetCamera();
}

void MainWindow::slotClearSecondaryMeshes()
{

}

void MainWindow::slotChangeToDeformer(){

}

void MainWindow::slotChangeToAnnotator(){

}

void MainWindow::slotChangeToSlicer(){

}

void MainWindow::slotClose(){
    exit(0);
}

void MainWindow::slotGenerate()
{
    this->cage = new DrawableMesh(Utilities::generateCage(this->model, Utilities::VOLUMETRIC));
    this->cageLoaded = true;
    this->cage->setIsCage(true);
    this->cage->setDrawWireframe(true);
    this->cage->setDrawPoints(true);
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->cagePoints->SetPoints(this->cage->getPoints());
    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(cagePoints);
    idFilter->PointIdsOn();
    idFilter->SetPointIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkPolyData* input = static_cast<vtkPolyData*>(idFilter->GetOutput());
    this->selectionStyle->SetCurrentRenderer(ren);
    this->selectionStyle->setPoints(input);
    this->selectionStyle->setAssembly(this->meshAssembly);
    this->selectionStyle->setCage(this->cage);
    this->selectionStyle->setSelectedPoints(&(this->pointsSelectionStatus));
    this->selectionStyle->setQvtkwidget(this->ui->qvtkWidget);
    this->selectionStyle->resetSelection();

    soModelPoints->resize(model->V.numels(), 3);
    soCagePoints = new ShapeOp::MatrixX3();
    soCagePoints->resize(cage->V.numels(), 3);
    unsigned int i = 0;
    for (IMATI_STL::Node* n = model->V.head(); n != nullptr; n = n->next()) {
        Vertex* v = static_cast<Vertex*>(n->data);
        ShapeOp::Vector3 p(v->x, v->y, v->z);
        soModelPoints->row(i++) = p;
    }
    i = 0;
    for (IMATI_STL::Node* n = cage->V.head(); n != nullptr; n = n->next()) {
        Vertex* v = static_cast<Vertex*>(n->data);
        soCagePoints->row(i++) = ShapeOp::Vector3(v->x, v->y, v->z);
    }

    this->deformationStyle->setData(input);
    this->deformationStyle->setModel(this->model);
    this->deformationStyle->setCage(this->cage);
    this->deformationStyle->setAssembly(this->meshAssembly);
    this->deformationStyle->setSelectedPoints(&(this->pointsSelectionStatus));
    this->deformationStyle->setCoordsComputed(false);
    this->deformationStyle->setQvtkwidget(this->ui->qvtkWidget);
    this->deformationStyle->setModelPoints(soModelPoints);
    this->deformationStyle->setCagePoints(soCagePoints);

    this->ui->actionVerticesSelection->setEnabled(true);
    this->ui->actionVerticesDeselection->setEnabled(true);
    this->ui->actionMeshDeformation->setEnabled(true);
    this->ui->actionStretch->setEnabled(true);
    //this->ui->actionGreen->setEnabled(true);
    this->ui->actionMean->setEnabled(true);
    this->ui->actionShowCage->setEnabled(true);
    this->ui->actionShowCage->setChecked(true);
    this->ui->cageVerticesLabel->setText(this->ui->cageVerticesLabel->text() + QString::number(cage->V.numels()));
    this->ui->cageEdgesLabel->setText(this->ui->cageEdgesLabel->text() + QString::number(cage->E.numels()));
    this->ui->cageTrianglesLabel->setText(this->ui->cageTrianglesLabel->text() + QString::number(cage->T.numels()));

    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotComputeCoords(){

    this->computeCoords();
    this->deformationStyle->setSoCoords(soCoords);
    this->deformationStyle->setCoordsComputed(true);

    associateCageVerticesToAnnotations();
}

void MainWindow::slotShowMesh(bool value){

    model->setDrawable(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshSurface(bool value)
{
    model->setDrawSurface(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshPoints(bool value)
{
    model->setDrawPoints(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshAnnotations(bool value)
{
    model->setDrawAnnotations(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshWireframe(bool value)
{
    this->model->setDrawWireframe(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCage(bool value){

    cage->setDrawable(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCageSurface(bool value)
{
    this->cage->setDrawSurface(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCageWireframe(bool value)
{
    this->cage->setDrawWireframe(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCagePoints(bool value)
{
    this->cage->setDrawPoints(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotSlicerPalette(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->sui->setVisible(value);
    this->ui->actionSlicerPalette->setChecked(value);
    if(value)
    {
        this->lui->setVisible(false);
        this->iui->setVisible(false);
        this->ui->actionLayerDialog->setChecked(false);
        this->ui->actionInfoPalette->setChecked(false);
        updateIntersection();
    }
    updateView();
}


void MainWindow::slotInfoPalette(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->iui->setVisible(value);
    this->ui->actionInfoPalette->setChecked(!value);
    if(value)
    {
        this->sui->setVisible(false);
        this->lui->setVisible(false);
        this->ui->actionSlicerPalette->setChecked(false);
        this->ui->actionLayerDialog->setChecked(false);
    }
    updateView();
}

void MainWindow::slotLayerDialog(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->lui->setVisible(value);
    this->ui->actionLayerDialog->setChecked(value);
    if(value)
    {
        this->sui->setVisible(false);
        this->iui->setVisible(false);
        this->ui->actionSlicerPalette->setChecked(false);
        this->ui->actionInfoPalette->setChecked(false);
    }
    updateView();
}



void MainWindow::slotCheckConstraints()
{
    if(constrained){/*

        for(unsigned int i = 0; i < closenessConstraintsID.size(); i++){
            auto c = solver->getConstraint(closenessConstraintsID[i]);
            dynamic_cast<ShapeOp::ClosenessConstraint*>(c.get())->setWeight(0);
        }*/


        this->solver->solve(100);

        this->ren->RemoveActor(meshAssembly);
        for(unsigned int i = 0; i < soCagePoints->rows(); i++){
            double pos[3] = {(*soCagePoints)(i, 0), (*soCagePoints)(i, 1), (*soCagePoints)(i, 2)};
            this->cage->setPointPosition(i, pos);
        }

        this->cage->setMeshModified(true);
        this->cage->update();
        this->cage->draw(meshAssembly);
        this->soCoords->deform();
        ShapeOp::Matrix3X mpt;
        mpt.resize(3, soModelPoints->rows());
        for(unsigned int i = 0; i < soModelPoints->rows(); i++){
            ShapeOp::Vector3 pos = soModelPoints->row(i);
            mpt.col(i) = pos;
        }

        this->solver->setModelPointsTransposed(mpt);
        this->model->setMeshModified(true);
        this->model->update();
        this->model->draw(meshAssembly);
        this->meshAssembly->Modified();
        this->ren->AddActor(meshAssembly);
        this->ui->qvtkWidget->update();
        this->updateView();

        std::cout << "Errors list: [" << std::endl;
        for(unsigned int i = 0; i < semanticConstraints.size(); i++){
            std::cout << "Semantic constraints n°" << i << ": ";
            std::cout << "Constraint error: " << semanticConstraints[i]->getError() << std::endl;
        }

        std::cout << "]" << std::endl<< std::endl << std::flush;
    }
}


void MainWindow::clusterSlices()
{

    if(!clusteringDone){
        points.clear();
        clusteringSlices.clear();
        vector<pair<Slice*, double> > cS = *Utilities::sliceMesh(model, SLICES_NUMBER);
        for(vector<pair<Slice*, double> >::iterator sit = cS.begin(); sit != cS.end(); sit++){
            Slice* s = static_cast<pair<Slice*, double> >(*sit).first;
            points.push_back(s->getFeatureVector());
        }

        for(unsigned int i = 0; i < cS.size(); i++){
            for(unsigned int j = i; j < cS.size(); j++){
                if(cS[i].second > cS[j].second){
                    pair<Slice*, double> p = cS[i];
                    cS[i] = cS[j];
                    cS[j] = p;
                    Eigen::VectorXd a = points[i];
                    points[i] = points[j];
                    points[j] = a;
                }

            }
        }

        for(unsigned int i = 0; i < cS.size(); i++){
            clusteringSlices.push_back(cS[i].first);
        }
        clusteringDone = true;
    }


    clusters = Utilities::kMeansCluster(points, clusterNumber);

    clusterChanged = false;
}

void MainWindow::resetCamera()
{
    vtkSmartPointer<vtkCamera> newCamera = vtkSmartPointer<vtkCamera>::New();
    newCamera->SetPosition(initialCamera->GetPosition());
    newCamera->SetFocalPoint(initialCamera->GetFocalPoint());
    newCamera->SetViewUp(initialCamera->GetViewUp());
    ren->SetActiveCamera(newCamera);
    ren->ResetCamera();
    ren->GetRenderWindow()->Render();
    this->ui->qvtkWidget->update();
}

void MainWindow::clearAll()
{
    clear();
    clearMesh();
    clearCage();
    clearSelections();
    clearConstraints();
    clearCoordinates();
    clearSecondaryMeshes();
    clearCamera();
    vtkSmartPointer<vtkAreaPicker> picker = vtkSmartPointer<vtkAreaPicker>::New();
    this->ui->qvtkWidget->renderWindow()->GetInteractor()->SetPicker(picker);
}

void MainWindow::clearMesh()
{
    this->meshLoaded = false;
    this->firstFragmentPositioned = false;
    this->lui->removeMainMesh(model);
    if(model != nullptr){
        this->meshAssembly->RemovePart(model->getCanvas());
        this->meshAssembly->Modified();
        delete model;
    }
    model = nullptr;
    modelCanvas = vtkSmartPointer<vtkPropAssembly>::New();
    if(soModelPoints != nullptr)
        delete  soModelPoints;
    soModelPoints = nullptr;
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->pointsSelectionStatus.clear();
    this->ui->actionShowMesh->setChecked(true);
    this->ui->actionShowMeshSurface->setChecked(true);
    this->ui->actionShowMeshWireframe->setChecked(false);
    this->ui->actionShowMeshPoints->setChecked(false);
    this->ui->actionDelete->setChecked(false);
    this->ui->actionLineSelection->setChecked(false);
    this->ui->actionRectSelection->setChecked(false);
    this->ui->actionLassoSelection->setChecked(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->qvtkWidget->update();
    clearSelections();
    clearCoordinates();
}

void MainWindow::clearCage()
{
    this->cageLoaded = false;
    this->firstFragmentPositioned = false;
    this->lui->removeCage(cage);
    if(this->cage != nullptr){
        this->meshAssembly->RemovePart(cage->getCanvas());
        delete this->cage;
    }
    this->cage = nullptr;
    if(soCagePoints != nullptr)
        delete  soCagePoints;
    soCagePoints = nullptr;
    this->lui->removeCage(cage);
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->deformationStyle = vtkSmartPointer<MeshDeformationStyle>::New();
    this->deformationStyle->setInfoPalette(iui);
    this->pointsSelectionStatus.clear();
    this->ui->actionShowCage->setChecked(true);
    this->ui->actionShowCageSurface->setChecked(false);
    this->ui->actionShowCageWireframe->setChecked(true);
    this->ui->actionShowCagePoints->setChecked(true);
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    this->meshAssembly->Modified();
    this->ui->qvtkWidget->update();
    clearCoordinates();

}

void MainWindow::clearCoordinates()
{
    if(this->soCoords != nullptr)
        delete soCoords;
    soCoords = nullptr;
    this->coordsComputed = false;
    this->firstFragmentPositioned = false;
    this->lui->setBcComputed(false);
}

void MainWindow::clearSelections()
{
    this->ui->qvtkWidget->update();
}

void MainWindow::clearConstraints()
{
    this->constrained = false;
    this->deformationStyle->setConstrained(this->constrained);
}

void MainWindow::clearCamera()
{
    ren->RemoveAllViewProps();
    this->resetCamera();
    this->ui->qvtkWidget->update();
}

void MainWindow::clearSecondaryMeshes()
{
    for(unsigned int i = 0; i < fragments.size(); i++){
        DrawableMesh* mesh = fragments[i];
        fragments.erase(fragments.begin() + i);
        this->lui->removeFragment(fragments[i]);
        this->fragmentsNames.erase(fragmentsNames.begin() + i);
        this->soFragmentsPoints.erase(soFragmentsPoints.begin() + i);
        delete mesh;
    }

    this->ui->qvtkWidget->update();
}

void MainWindow::associateCageVerticesToAnnotations()
{
    int flagValue = 12;
    for(IMATI_STL::Node* n = this->cage->V.head(); n != nullptr; n = n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;

    for(unsigned int i = 0; i < this->model->getAnnotations().size(); i++)
    {
        std::vector<IMATI_STL::Vertex*> involved = this->model->getAnnotations()[i]->getInvolvedVertices();
        std::vector<unsigned int> associatedCageVertices;
        for(unsigned int j = 0; j < involved.size(); j++)
        {
            std::vector<unsigned int> cageVertices = this->soCoords->getMaxInfluenceCageVertices(this->model->getPointId(involved[j]));
            for(unsigned int k = 0; k < cageVertices.size(); k++)
            {
                Vertex* v = this->cage->getPoint(cageVertices[k]);
                if(v->info == nullptr)
                {
                    v->info = new int(flagValue);
                    associatedCageVertices.push_back(cageVertices[k]);
                }

            }
        }
        this->annotationToCage[i] = associatedCageVertices;

        for(unsigned int j = 0; j < associatedCageVertices.size(); j++)
            this->cage->getPoint(associatedCageVertices[j])->info = nullptr;
    }

    for(IMATI_STL::Node* n = this->cage->V.head(); n != nullptr; n = n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;
}

void MainWindow::slotClearAll(){

    clearAll();
    init();
    drawInitialTetrahedron();

}

void MainWindow::slotModel(bool checked){

    if(!checked)
        model->setDrawable(false);
    else
        model->setDrawable(true);

    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();

}

void MainWindow::slotCamera(bool checked){

    if(checked){

        vtkSmartPointer<vtkInteractorStyleTrackballCamera> cameraStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();        iren->SetInteractorStyle(cameraStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);

    }

}

void MainWindow::slotVerticesSelection(bool checked){

    if(checked){

        this->iren->SetInteractorStyle(selectionStyle);
        this->selectionStyle->SetCurrentRenderer(ren);
        this->selectionStyle->selectionMode = true;
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionCamera->setChecked(false);

    }else
        this->ui->actionVerticesSelection->setChecked(true);
}

void MainWindow::slotVerticesDeselection(bool checked){

    if(checked){

        iren->SetInteractorStyle(selectionStyle);
        selectionStyle->selectionMode = false;
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionCamera->setChecked(false);

    }else
        this->ui->actionVerticesDeselection->setChecked(true);
}

void MainWindow::slotMeshDeformation(bool checked){

    if(checked){
        deformationStyle->setStretch(false);
        iren->SetInteractorStyle(deformationStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionCamera->setChecked(false);
    }

}

void MainWindow::slotStretch(bool checked)
{
    if(checked){
        deformationStyle->setStretch(true);
        this->ui->qvtkWidget->update();
        iren->SetInteractorStyle(deformationStyle);
        this->ui->actionCamera->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
    }
}

void MainWindow::computeCoords(){

    switch(coordType){
        case(GC):
            throw("At the moment it is not possible to use Green Coordinates.");
        case(MVC):
        default:
            this->soCoords = new ShapeOpMeanValueCoordinates(model, cage, soModelPoints, soCagePoints);
    }

    this->soCoords->computeCoordinates();
    this->modelToCage.clear();
    for(IMATI_STL::Node* n = this->model->V.head(); n != nullptr; n = n->next())
    {
        Vertex* v = static_cast<Vertex*>(n->data);
        this->modelToCage[this->model->getPointId(v)] = this->soCoords->getMaxInfluenceCageVertices(this->model->getPointId(v));
    }
    this->coordsComputed = true;
    this->lui->setBcComputed(true);

}

void MainWindow::meshLoadedPreparation(){


    QFileInfo info(modelFilename);
    setWindowTitle(info.fileName() + " ― MainWindow");
    this->meshLoaded = true;
    this->model->setIsTemplate(true);
    this->model->setDrawSurface(true);
    this->model->setDrawWireframe(false);
    this->model->setDrawPoints(false);
    //Utilities::computeMeshVolume(model);

    this->soModelPoints = new ShapeOp::MatrixX3();
    this->soModelPoints->resize(model->V.numels(), 3);
    this->lui->addMainMesh(modelFilename, model);
    unsigned int i = 0;
    for(IMATI_STL::Node* n = model->V.head(); n != nullptr; n = n->next()){
        Vertex* v = static_cast<Vertex*>(n->data);
        ShapeOp::Vector3 p = {v->x, v->y, v->z};
        this->soModelPoints->row(i++) = p;
    }

    this->modelPoints->SetPoints(this->model->getPoints());
    this->modelTriangles->SetPoints(this->model->getPoints());
    this->modelTriangles->SetPolys(this->model->getTriangles());
    this->modelTriangles->BuildCells();
    this->modelTriangles->BuildLinks();

    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(modelTriangles);
    idFilter->PointIdsOn();
    idFilter->SetPointIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkSmartPointer<vtkIdFilter> idFilterMesh = vtkSmartPointer<vtkIdFilter>::New();
    idFilterMesh->SetInputData(modelPoints);
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetCellIdsArrayName("OriginalMeshIds");
    idFilterMesh->Update();

    vtkSmartPointer<vtkPolyData> inputMesh = static_cast<vtkPolyData*>(idFilterMesh->GetOutput());
    vtkSmartPointer<vtkPolyData> input = static_cast<vtkPolyData*>(idFilter->GetOutput());

    this->ui->actionCamera->setEnabled(true);
    this->ui->actionDelete->setEnabled(true);
    this->ui->actionLineSelection->setEnabled(true);
    this->ui->actionRectSelection->setEnabled(true);
    this->ui->actionLassoSelection->setEnabled(true);
    this->ui->actionAnnotation->setEnabled(true);
    this->ui->actionAddVerticesRectangle->setEnabled(true);
    this->ui->actionShowMesh->setEnabled(true);
    this->ui->actionShowMesh->setChecked(true);
    this->ui->actionSlicerPalette->setEnabled(true);
    this->ui->actionInfoPalette->setEnabled(true);
    this->ui->verticesLabel->setText("Vertices: " + QString::number(model->V.numels()));
    this->ui->edgesLabel->setText("Edges: " + QString::number(model->E.numels()));
    this->ui->trianglesLabel->setText("Triangles: " + QString::number(model->T.numels()));
    this->meshAssembly->RemovePart(visualPropertiesAssembly);
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->model->setMeshModified(true);
    this->model->setAnnotationsModified(true);
    this->model->update();
    this->model->draw(meshAssembly);
    this->meshAssembly->AddPart(visualPropertiesAssembly);
    this->ren->AddActor(meshAssembly);
    this->ren->Modified();
    this->ui->qvtkWidget->update();
    double corner[3], min[3], mid[3], max[3], sizes[3];
    vtkOBBTree::ComputeOBB(this->model->getPoints(), corner, max, mid, min, sizes);

    ExtendedTrimesh* obb = new ExtendedTrimesh();
    IMATI_STL::Point minAx(min[0], min[1] , min[2] );
    IMATI_STL::Point midAx(mid[0], mid[1] , mid[2] );
    IMATI_STL::Point maxAx(max[0], max[1] , max[2] );
    Vertex* d = obb->newVertex(corner[0], corner[1], corner[2]);
    Vertex* c = obb->newVertex();
    c->setValue(*d + minAx);
    Vertex* e = obb->newVertex();
    e->setValue(*d + midAx);
    Vertex* a = obb->newVertex();
    a->setValue(*d + maxAx);
    Vertex* b = obb->newVertex();
    b->setValue(*a + minAx);
    Vertex* f = obb->newVertex();
    f->setValue(*a + midAx);
    Vertex* g = obb->newVertex();
    g->setValue(*b + midAx);
    Vertex* h = obb->newVertex();
    h->setValue(*c + midAx);
    obb->V.appendTail(a);
    obb->V.appendTail(b);
    obb->V.appendTail(c);
    obb->V.appendTail(d);
    obb->V.appendTail(e);
    obb->V.appendTail(f);
    obb->V.appendTail(g);
    obb->V.appendTail(h);
    ExtVertex* a_ = new ExtVertex(a);
    ExtVertex* b_ = new ExtVertex(b);
    ExtVertex* c_ = new ExtVertex(c);
    ExtVertex* d_ = new ExtVertex(d);
    ExtVertex* e_ = new ExtVertex(e);
    ExtVertex* f_ = new ExtVertex(f);
    ExtVertex* g_ = new ExtVertex(g);
    ExtVertex* h_ = new ExtVertex(h);
    obb->CreateTriangleFromVertices(a_, d_, f_);
    obb->CreateTriangleFromVertices(a_, f_, g_);
    obb->CreateTriangleFromVertices(a_, g_, b_);
    obb->CreateTriangleFromVertices(a_, b_, d_);
    obb->CreateTriangleFromVertices(b_, c_, d_);
    obb->CreateTriangleFromVertices(b_, h_, c_);
    obb->CreateTriangleFromVertices(b_, g_, h_);
    obb->CreateTriangleFromVertices(c_, e_, d_);
    obb->CreateTriangleFromVertices(c_, h_, e_);
    obb->CreateTriangleFromVertices(d_, e_, f_);
    obb->CreateTriangleFromVertices(e_, h_, f_);
    obb->CreateTriangleFromVertices(f_, h_, g_);
    obb->mergeCoincidentEdges();
    obb->checkGeometry();
    obb->checkConnectivity();
    obb->savePLY("teapot_bb_cage.ply");
    ren->ResetCamera();
    this->totalWidth = ((*c) - (*d)).length();
    this->totalHeight = ((*a) - (*d)).length();
    this->totalDepth = ((*e) - (*d)).length();
    translationVector(0) = a->x + (50 * totalWidth) / 100;
    translationVector(1) = d->y + (50 * totalHeight) / 100;
    translationVector(2) = d->z + (50 * totalDepth) / 100;
    translationMatrix = Utilities::translationMatrix(translationVector);
    this->sui->getTabWidget()->setEnabled(true);
    sliceRen->GetActiveCamera()->SetPosition(center.x, center.y + 2 * totalHeight, center.z);
    sliceRen->GetActiveCamera()->SetViewUp(0, 0, 1);
    sliceRen->GetActiveCamera()->SetFocalPoint(center.x, center.y, center.z);


}

void MainWindow::slotMeanValue(){
    coordType = MVC;
}

void MainWindow::slotGreen(){
    coordType = GC;
}

void MainWindow::slotVisible(bool checked){

    this->selectionStyle->setVisiblePointsOnly(checked);

}

void MainWindow::slotMeshColor(){

    int r,g,b;
    QColor meshColor = QColorDialog::getColor();
    meshColor.getRgb(&r,&g,&b);
    this->model->setColor(r,g,b);
    this->model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotCageColor(){

    int r,g,b;
    QColor cageColor = QColorDialog::getColor();
    cageColor.getRgb(&r,&g,&b);
    this->cage->setColor(r,g,b);
    this->cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();

}

void MainWindow::slotAddAnnotationsConstraint(std::string type, double weight, double minValue, double maxValue, unsigned int measureId1, unsigned int measureId2, bool directed)
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
    constraint->setPoints(soModelPoints);
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
            model->addAnnotationsRelationship(constraint->getAnnotations()[i], constraint->getAnnotations()[j], type, weight, directed);
        }
    }

    this->model->setMeshModified(true);
    this->model->update();
    this->model->draw(this->meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotConstrainRelationship(AnnotationsRelationship *&relationship)
{
    AnnotationsRelationship* r = relationship;
    AnnotationsConstraint* constraint = new AnnotationsConstraint(relationship);
    constraint->setPoints(soModelPoints);
    constraint->constrain();
    std::cout << constraint << " " << relationship << std::endl << std::flush;
    relationship = constraint;
    delete r;
    semanticConstraints.push_back(constraint);
    this->model->setMeshModified(true);
    this->model->update();
    this->model->draw(this->meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotConstrain()
{
    this->closenessConstraintsID.clear();

    this->solver = new ConstraintSolver();
    for (unsigned int i = 0; i < semanticConstraints.size(); i++) {
        semanticConstraints[i]->setSolver(solver);
        vector<shared_ptr<ShapeOp::Constraint> > constraints = semanticConstraints[i]->getConstraints();
        for(unsigned int j = 0; j < constraints.size(); j++){
            auto c = shared_ptr<ShapeOp::Constraint>(constraints[j]);
            this->solver->addConstraint(c);
            this->constraints.push_back(c);
        }
    }

    ShapeOp::Matrix3X modelPointsTransposed = soModelPoints->transpose();
    double weight = 1.0;

    for (int i = 0; i < model->V.numels(); i++) {
        Vertex* v = model->getPoint(static_cast<unsigned long>(i));
        int vid = i;
        vector<int> id0 = { vid };
        auto c = make_shared<ShapeOp::ClosenessConstraint>(id0, weight, modelPointsTransposed);
        this->constraints.push_back(c);
        closenessConstraintsID.push_back(static_cast<unsigned int>(solver->addConstraint(c)));
//        vector<int> vvids = {vid};
//        for (IMATI_STL::Node* n1 = v->VV()->head(); n1 != nullptr; n1 = n1->next())
//            vvids.push_back(static_cast<int>(model->getPointId(static_cast<Vertex*>(n1->data))));
//        auto l = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", vvids, 12*weight, modelPointsTransposed);
//        this->constraints.push_back(l);
    }

    this->deformationStyle->setClosenessConstraintsID(closenessConstraintsID);

    this->constrained = true;
    this->lui->setConstraintsImposed(true);
    this->deformationStyle->setConstrained(constrained);

    this->solver->setModelPointsTransposed(modelPointsTransposed);

    this->solver->setCagePoints(soCagePoints);
    this->solver->setBarycentricCoordinates(this->soCoords->getCoordinates());
    if(!this->solver->initialize()){
        std::cout << "Non symmetric or non positive-definite!" << std::endl << std::flush;
        exit(4);
    }

    this->deformationStyle->setSolver(solver);
    this->deformationStyle->setSemanticConstraints(semanticConstraints);
    this->deformationStyle->setConstraints(constraints);
    this->iui->setConstraints(semanticConstraints);
    this->iui->updateInfo();
}

void MainWindow::slotConstrainSystem(std::vector<AnnotationsConstraint*> semanticConstraints)
{
    this->solver = new ConstraintSolver();
    this->semanticConstraints = semanticConstraints;
    ShapeOp::Matrix3X modelPointsTransposed = soModelPoints->transpose();
    double weight = 1.0;

    for (int i = 0; i < model->V.numels(); i++) {
        int vid = i;
        vector<int> id0 = { vid };
        auto c = make_shared<ShapeOp::ClosenessConstraint>(id0, weight, modelPointsTransposed);
        this->constraints.push_back(c);
        closenessConstraintsID.push_back(static_cast<unsigned int>(this->solver->addConstraint(c)));
//        vector<int> vvids = {vid};
//        for (IMATI_STL::Node* n1 = v->VV()->head(); n1 != nullptr; n1 = n1->next())
//            vvids.push_back(static_cast<int>(model->getPointId(static_cast<Vertex*>(n1->data))));
//        auto l = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", vvids, 12*weight, modelPointsTransposed);
//        this->constraints.push_back(l);
    }

    for (unsigned int i = 0; i < semanticConstraints.size(); i++) {
        semanticConstraints[i]->setSolver(solver);
        std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints = semanticConstraints[i]->getConstraints();
        for(unsigned int j = 0; j < constraints.size(); j++){
            auto c = std::shared_ptr<ShapeOp::Constraint>(constraints[j]);
            this->solver->addConstraint(c);
            this->constraints.push_back(c);
        }
    }

    this->deformationStyle->setClosenessConstraintsID(closenessConstraintsID);

    this->constrained = true;
    this->lui->setConstraintsImposed(true);
    this->deformationStyle->setConstrained(constrained);

    this->solver->setModelPointsTransposed(modelPointsTransposed);

    this->solver->setCagePoints(soCagePoints);
    this->solver->setBarycentricCoordinates(this->soCoords->getCoordinates());
    if(!this->solver->initialize()){
        std::cout << "Non symmetric or non positive-definite!" << std::endl << std::flush;
        exit(4);
    }

    this->deformationStyle->setSolver(this->solver);
    this->deformationStyle->setSemanticConstraints(semanticConstraints);
    this->deformationStyle->setConstraints(constraints);
    this->iui->setConstraints(semanticConstraints);
    this->iui->updateInfo();
}

void MainWindow::slotReleaseSystem()
{
    if(this->solver != nullptr)
        delete this->solver;
    this->constraints.clear();
    this->constrained = false;
    this->lui->setConstraintsImposed(false);
    this->deformationStyle->setConstrained(constrained);
    this->iui->setConstraints(semanticConstraints);
    this->iui->updateInfo();
}

void MainWindow::slotSubstituteMesh(DrawableMesh *original, DrawableMesh *newMesh, QString filename)
{
    meshAssembly->RemovePart(original->getCanvas());
    if(original == model)
    {
        lui->removeMainMesh(original);
        delete original;
        model = newMesh;
        this->meshLoadedPreparation();
        this->ui->verticesLabel->setText(QString::fromStdString("Vertices: ") + QString::number(model->V.numels()));
        this->ui->edgesLabel->setText(QString::fromStdString("Edges: ") + QString::number(model->E.numels()));
        this->ui->trianglesLabel->setText(QString::fromStdString("Triangles: ") + QString::number(model->T.numels()));
    }
    else
    {
        lui->removeFragment(original);
        unsigned int pos = static_cast<unsigned int>(std::find(fragments.begin(), fragments.end(), original) - fragments.begin());
        fragments[pos] = newMesh;
        fragmentsNames[pos] = filename;
        ShapeOp::MatrixX3* tmp = new ShapeOp::MatrixX3();
        tmp->resize(newMesh->V.numels(), 3);
        unsigned int i = 0;
        for(IMATI_STL::Node* n = newMesh->V.head(); n != nullptr; n = n->next())
        {
            IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
            ShapeOp::Vector3 point = {v->x, v->y, v->z};
            tmp->row(i++) = point;
        }
        *soFragmentsPoints[pos] = *tmp;
        this->meshAssembly->RemovePart(original->getCanvas());
        this->meshAssembly->Modified();
        this->ren->Modified();
        this->ui->qvtkWidget->update();
        this->updateView();
        delete original;
        lui->addFragment(filename, newMesh);

        newMesh->setMeshModified(true);
        newMesh->setAnnotationsModified(true);
        newMesh->update();
        newMesh->draw(meshAssembly);
    }
    meshAssembly->RemovePart(newMesh->getCanvas()); //This because in AnnotationWindow the canvas is overridden just after.
    resetCamera();
}

void MainWindow::on_xSlider_valueChanged(int value){

    xRotationMatrix = Utilities::xRotationMatrix(M_PI * value / 100);
    updateIntersection();

}

void MainWindow::on_ySlider_valueChanged(int value){

    yRotationMatrix = Utilities::yRotationMatrix(M_PI * value / 100);
    updateIntersection();

}

void MainWindow::on_zSlider_valueChanged(int value){

    zRotationMatrix = Utilities::zRotationMatrix(M_PI * value / 100);
    updateIntersection();

}

void MainWindow::on_showNodesBox_stateChanged(int value)
{
    if(value)
        this->showNodes = true;
    else
        this->showNodes = false;
    this->updateView();
}

void MainWindow::on_showArcNodesBox_stateChanged(int value)
{
    if(value)
        this->showArcNodes = true;
    else
        this->showArcNodes = false;
    this->updateView();
}

void MainWindow::on_levelOfDetailSlider_valueChanged(int value)
{
    this->levelOfDetail = MAX_LOD - value;
    updateView();
}

void MainWindow::on_maxErrorSlider_valueChanged(int value)
{
    this->allowedError = MAX_ERROR * static_cast<double>(value) / 100.0;
    updateIntersection();
}

void MainWindow::on_showSlices_stateChanged(int value)
{
    this->showSliceOnMesh = value;
    this->updateView();
}

void MainWindow::on_showFittedCircle_stateChanged(int value)
{
    this->showFittedCircle = value;
    this->updateView();
}

void MainWindow::on_showSkeleton_stateChanged(int value)
{
    this->showSkeleton = value;
    this->updateView();
}

void MainWindow::on_showConvexHull_stateChanged(int value)
{
    this->showConvexHull = value;
    this->updateView();
}

void MainWindow::slotUpdateMeshView(DrawableMesh * mesh)
{
    mesh->draw(meshAssembly);
    ren->Modified();
    this->ui->qvtkWidget->update();
}

void MainWindow::slotDeleteMesh(DrawableMesh *mesh)
{
    if(mesh == model)
        clearMesh();
    else if(mesh == cage)
        clearCage();
    else{
        unsigned int pos = static_cast<unsigned int>(std::find(fragments.begin(), fragments.end(), mesh) - fragments.begin());
        fragments.erase(fragments.begin() + pos);
        fragmentsNames.erase(fragmentsNames.begin() + pos);
        ShapeOp::MatrixX3* tmp = soFragmentsPoints[pos];
        soFragmentsPoints.erase(soFragmentsPoints.begin() + pos);
        delete tmp;
        this->meshAssembly->RemovePart(mesh->getCanvas());
        delete mesh;
        this->meshAssembly->Modified();
        this->ren->Modified();
        this->ui->qvtkWidget->update();
        this->updateView();
    }
}

void MainWindow::slotEditAnnotations(DrawableMesh *mesh)
{
    meshAssembly->RemovePart(mesh->getCanvas());
    if(ren != nullptr && mesh != nullptr)
    {
        for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
            for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
                meshAssembly->RemovePart(dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->getCanvas());

        meshAssembly->Modified();
        this->ui->qvtkWidget->update();
    }
    AnnotationWindow* annWin = new AnnotationWindow(this);
    annWin->setCurrentPath(currentPath);
    annWin->setIsSystemConstrained(this->constrained);
    annWin->setCoordsComputed(coordsComputed);
    annWin->setMesh(mesh);
    if(mesh == model)
        annWin->setSoMeshPoints(soModelPoints);
    else if(mesh == cage)
        annWin->setSoMeshPoints(soCagePoints);
    else{
        unsigned int pos = static_cast<unsigned int>(std::find(fragments.begin(), fragments.end(), mesh) - fragments.begin());
        annWin->setSoMeshPoints(soFragmentsPoints[pos]);
    }
    annWin->setSemanticConstraints(semanticConstraints);
    annWin->show();
    connect(annWin, SIGNAL(annotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*>)), this, SLOT(slotAnnotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*>)));
    connect(annWin, SIGNAL(constrainSystem(std::vector<AnnotationsConstraint*>)), this, SLOT(slotConstrainSystem(std::vector<AnnotationsConstraint*>)));
    connect(annWin, SIGNAL(releaseConstraints()), this, SLOT(slotReleaseSystem()));
    connect(annWin, SIGNAL(constrainRelationship(AnnotationsRelationship*&)), this, SLOT(slotConstrainRelationship(AnnotationsRelationship*&)));
    connect(annWin, SIGNAL(substituteMesh(DrawableMesh*, DrawableMesh*, QString)), this, SLOT(slotSubstituteMesh(DrawableMesh*, DrawableMesh*, QString)));
    connect(annWin, SIGNAL(currentPathChanged(std::string)), this, SLOT(slotCurrentPathChanged(std::string)));
}

void MainWindow::slotFitRigidly(DrawableMesh * mesh)
{
    CorrespondencesDialog* cd = new CorrespondencesDialog(this);
    cd->setTemplateMesh(model);
    cd->setFragmentMesh(mesh);
    cd->updateView();
    cd->exec();
    std::vector<std::pair<int, int> > correspondences = cd->getCorrespondences();
    delete cd;

    if(correspondences.size() > 0){
        //Rigid registration component: Umeyama algorithm
        ShapeOp::Matrix3X src, tgt;
        src.resize(3, static_cast<long>(correspondences.size()));
        tgt.resize(3, static_cast<long>(correspondences.size()));
        for(unsigned int i = 0; i < correspondences.size(); i++){
            Vertex* srcPt = model->getPoint(static_cast<unsigned long>(correspondences[i].first));
            Vertex* tgtPt = mesh->getPoint(static_cast<unsigned long>(correspondences[i].second));
            src.col(i) = ShapeOp::Vector3(srcPt->x, srcPt->y, srcPt->z);
            tgt.col(i) = ShapeOp::Vector3(tgtPt->x, tgtPt->y, tgtPt->z);
        }
        ShapeOp::Matrix44 transformation = Eigen::umeyama(src, tgt, !firstFragmentPositioned);
        if(!firstFragmentPositioned && fitTemplateOnFragments)
        {

            ShapeOp::MatrixXX transformedModelPoints = soModelPoints->transpose();
            ShapeOp::MatrixXX transformedCagePoints = soCagePoints->transpose();
            transformedModelPoints.conservativeResize(transformedModelPoints.rows() + 1, transformedModelPoints.cols());
            transformedModelPoints.row(transformedModelPoints.rows() - 1).setOnes();
            transformedCagePoints.conservativeResize(transformedCagePoints.rows() + 1, transformedCagePoints.cols());
            transformedCagePoints.row(transformedCagePoints.rows() - 1).setOnes();
            transformedModelPoints = transformation * transformedModelPoints;
            transformedCagePoints = transformation * transformedCagePoints;
            transformedModelPoints.conservativeResize(transformedModelPoints.rows() - 1, transformedModelPoints.cols());
            transformedCagePoints.conservativeResize(transformedCagePoints.rows() - 1, transformedCagePoints.cols());
            *soModelPoints = transformedModelPoints.transpose();
            *soCagePoints = transformedCagePoints.transpose();
            for(unsigned int i = 0; i < model->V.numels(); i++){
                ShapeOp::Vector3 tmp = soModelPoints->row(i);
                double p[3] = {tmp(0), tmp(1), tmp(2)};
                model->setPointPosition(i, p);
            }
            for(unsigned int i = 0; i < static_cast<unsigned int>(cage->V.numels()); i++){
                ShapeOp::Vector3 tmp = soCagePoints->row(i);
                double p[3] = {tmp(0), tmp(1), tmp(2)};
                cage->setPointPosition(i, p);
            }
            this->model->setMeshModified(true);
            this->model->update();
            this->model->draw(meshAssembly);
            this->cage->setMeshModified(true);
            this->cage->update();
            this->cage->draw(meshAssembly);
            firstFragmentPositioned = true;
        } else {
            int pos = static_cast<int>(std::find(fragments.begin(), fragments.end(), mesh) - fragments.begin());
            ShapeOp::MatrixXX transformedFragmentPoints = soFragmentsPoints[static_cast<unsigned long>(pos)]->transpose();
            transformedFragmentPoints.conservativeResize(transformedFragmentPoints.rows() + 1, transformedFragmentPoints.cols());
            transformedFragmentPoints.row(transformedFragmentPoints.rows() - 1).setOnes();
            transformedFragmentPoints = transformation.inverse() * transformedFragmentPoints;
            transformedFragmentPoints.conservativeResize(transformedFragmentPoints.rows() - 1, transformedFragmentPoints.cols());
            *(soFragmentsPoints[static_cast<unsigned long>(pos)]) = transformedFragmentPoints.transpose();
            for(unsigned int i = 0; i < static_cast<unsigned int>(mesh->V.numels()); i++){
                ShapeOp::Vector3 tmp = soFragmentsPoints[static_cast<unsigned long>(pos)]->row(i);
                double p[3] = {tmp(0), tmp(1), tmp(2)};
                mesh->setPointPosition(i, p);
            }
            mesh->setMeshModified(true);
            mesh->update();
            mesh->draw(this->meshAssembly);
        }

//        std::ofstream stream;
//        stream.open("model.m");
//        if(stream.is_open())
//        {
//            stream << transformedModelPoints;
//            stream.close();
//        }
//        stream.open("fragment.m");
//        if(stream.is_open())
//        {
//            stream << transformedFragmentPoints;
//            stream.close();
//        }
//        stream.open("transformation.m");
//        if(stream.is_open())
//        {
//            stream << transformation;
//            stream.close();
//        }




//    for(unsigned int i = 0; i < model->getAnnotations().size(); i++)
//        for(unsigned int j = 0; j < model->getAnnotations()[i]->getAttributes().size(); j++)
//        {
//            DrawableBoundingMeasure* att = dynamic_cast<DrawableBoundingMeasure*>(model->getAnnotations()[i]->getAttributes()[j]);
//            if(att != nullptr)
//            {
//                IMATI_STL::Point* dir = att->getDirection();
//                IMATI_STL::Point* origin = att->getOrigin();
//                Eigen::Vector3d dirEig = {dir->x, dir->y, dir->z};
//                Eigen::Vector4d originEig = {origin->x, origin->y, origin->z, 1};
//                Eigen::Matrix4d tmpTransformation = transformation;
//                Eigen::Vector4d lastColumnNew = {0.0, 0.0, 0.0, 1.0};
//                originEig = transformation * originEig;
//                origin->setValue(originEig(0), originEig(1), originEig(2));
//                tmpTransformation.col(3) = lastColumnNew;
//                tmpTransformation.col(0) /= transformation.block(0,0,3,1).norm();
//                tmpTransformation.col(1) /= transformation.block(0,1,3,1).norm();
//                tmpTransformation.col(2) /= transformation.block(0,2,3,1).norm();
//                dirEig = tmpTransformation.block(0, 0, 3, 3) * dirEig;
//                dir->setValue(dirEig(0), dirEig(1), dirEig(2));
//                att->setDirection(dir);
//                att->update();
//            }
//        }


        this->ui->qvtkWidget->update();
        this->meshAssembly->Modified();
    }
}

void MainWindow::slotAdaptTemplate(DrawableMesh * mesh)
{

    NonRigidFitting wrapper(model, mesh);
    wrapper.setCoordinates(soCoords);
    wrapper.setSoTemplatePoints(soModelPoints);
    wrapper.setSoCagePoints(soCagePoints);
    wrapper.setCageMesh(cage);
    wrapper.setContext(this);
    wrapper.startFitting();

}

void MainWindow::slotAnnotationWindowClosed(DrawableMesh *mesh, std::vector<AnnotationsConstraint*> semanticConstraints)
{
    this->semanticConstraints = semanticConstraints;

    meshAssembly->RemovePart(mesh->getCanvas());
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
        {
            Attribute* a = mesh->getAnnotations()[i]->getAttributes()[j];
            meshAssembly->RemovePart(dynamic_cast<DrawableAttribute*>(a)->getCanvas());
            dynamic_cast<DrawableAttribute*>(a)->setRenderer(ren);
        }

    mesh->draw(meshAssembly);
    if(coordsComputed)
        associateCageVerticesToAnnotations();

    this->ui->qvtkWidget->update();
}

void MainWindow::slotCurrentPathChanged(string newPath)
{
    currentPath = newPath;
}

void MainWindow::on_xSlider2_valueChanged(int value){

    IMATI_STL::Point a, b;
    model->getBoundingBox(a, b);
    translationVector(0) = min(a.x, b.x) + (value * totalWidth) / 100;
    translationMatrix = Utilities::translationMatrix(translationVector);
    //center.y += (value - previousHeightValue) * totalHeight / 100;
    //previousHeightValue = value;
    updateIntersection();

}

void MainWindow::on_ySlider2_valueChanged(int value){

    IMATI_STL::Point a, b;
    model->getBoundingBox(a, b);
    translationVector(1) = min(a.y, b.y) + (value * totalHeight) / 100;
    translationMatrix = Utilities::translationMatrix(translationVector);
    //center.y += (value - previousHeightValue) * totalHeight / 100;
    //previousHeightValue = value;
    updateIntersection();

}

void MainWindow::on_zSlider2_valueChanged(int value){

    IMATI_STL::Point a, b;
    model->getBoundingBox(a, b);
    translationVector(2) = min(a.z, b.z) + (value * totalDepth) / 100;
    translationMatrix = Utilities::translationMatrix(translationVector);
    //center.y += (value - previousHeightValue) * totalHeight / 100;
    //previousHeightValue = value;
    updateIntersection();

}

void MainWindow::on_showBoundingBox_stateChanged(int value)
{
    this->showBoundingBox = value;
    this->updateView();
}

void MainWindow::on_clusterCheckBox_stateChanged(int value)
{
    this->showClusters = value;
    this->showSliceOnMesh = !value;
    if(value && (!clusteringDone || clusterChanged))
        clusterSlices();
    this->updateView();
}


void MainWindow::on_clusterNumberBox_valueChanged(int value)
{
    this->clusterNumber = static_cast<unsigned int>(value);
    this->clusterChanged = true;
    if(sui->getClusterCheckBox()->isChecked())
        clusterSlices();
    updateView();
}

void MainWindow::on_showBoundingRectangle_stateChanged(int value)
{
    this->showBoundingRectangle = value;
    this->updateView();
}

void MainWindow::on_showDiagonals_stateChanged(int value)
{
    this->showDiagonals = value;
    this->updateView();
}


void MainWindow::on_showCenters_stateChanged(int value)
{
    this->showCenters = value;
    this->updateView();
}
