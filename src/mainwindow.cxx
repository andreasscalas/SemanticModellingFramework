#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <vtkOBBTree.h>
#include <time.h>
#include <characterfittingwrapper.h>
#include <annotationwindow.h>
#include <arc.h>
#include <graph.h>
#include <annotationmeasuresconstraint.h>
#include <nonrigidfitting.h>
using namespace IMATI_STL;
using namespace std;

#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

vtkStandardNewMacro(MeshDeformationStyle)
vtkStandardNewMacro(AnnotationSelectionInteractorStyle)
vtkStandardNewMacro(CageVerticesSelectionStyle)
vtkStandardNewMacro(VerticesSelectionStyle)
vtkStandardNewMacro(TriangleSelectionStyle)
vtkStandardNewMacro(LineSelectionStyle)
vtkStandardNewMacro(DebugTriangleSelectionStyle)

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->init();
    this->write(initialInstruction);
    ad = new AnnotationDialog(this);
    cd = new AnnotationConstraintDialog(this);
    this->ui->line->setVisible(false);
    this->ui->genericPalette->setVisible(true);
    this->lui->setVisible(true);
    this->ui->actionLayerDialog->setChecked(true);
    iren = static_cast<QVTKInteractor*>(this->ui->qvtkWidget->GetRenderWindow()->GetInteractor());

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
    connect(this->ui->actionTransfer, SIGNAL(triggered(bool)), this, SLOT(slotTransfer()));
    connect(this->ui->actionVisible, SIGNAL(triggered(bool)), this, SLOT(slotVisible(bool)));
    connect(this->ui->actionSlicer, SIGNAL(triggered()), this, SLOT(slotChangeToSlicer()));
    connect(this->ui->actionCheckConstraints, SIGNAL(triggered()), this, SLOT(slotCheckConstraints()));
    connect(this->ui->actionConstrain, SIGNAL(triggered()), this, SLOT(slotConstrain()));
    connect(this->ui->actionSelectAnnotations, SIGNAL(triggered(bool)), this, SLOT(slotSelectAnnotations(bool)));
    connect(this->ui->actionAddAnnotationsConstraint, SIGNAL(triggered()), this, SLOT(slotActionAddAnnotationsConstraint()));
    connect(this->lui, SIGNAL(updateMeshView(DrawableMesh*)), this, SLOT(slotUpdateMeshView(DrawableMesh*)));
    connect(this->lui, SIGNAL(deleteMesh(DrawableMesh*)), this, SLOT(slotDeleteMesh(DrawableMesh*)));
    connect(this->lui, SIGNAL(editAnnotations(DrawableMesh*)), this, SLOT(slotEditAnnotations(DrawableMesh*)));
    connect(this->lui, SIGNAL(adaptTemplate(DrawableMesh*)), this, SLOT(slotAdaptTemplate(DrawableMesh*)));
    /*connect(this->lui, SIGNAL(openAnnotations(DrawableMesh*)), this, SLOT(slotOpenAnnotations(DrawableMesh*)));
    connect(this->lui, SIGNAL(removeAnnotations(DrawableMesh*)), this, SLOT(slotRemoveAnnotations(DrawableMesh*)));*//*
    connect(this->sui->getXSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_xSlider_valueChanged(int)));
    connect(this->sui->getYSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_ySlider_valueChanged(int)));
    connect(this->sui->getZSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_zSlider_valueChanged(int)));
    connect(this->sui->getXSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_xSlider2_valueChanged(int)));
    connect(this->sui->getYSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_ySlider2_valueChanged(int)));
    connect(this->sui->getZSlider2(), SIGNAL(valueChanged(int)), this, SLOT(on_zSlider2_valueChanged(int)));
    connect(this->sui->getMaxErrorSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_maxErrorSlider_valueChanged(int)));
    connect(this->sui->getLevelOfDetailSlider(), SIGNAL(valueChanged(int)), this, SLOT(on_levelOfDetailSlider_valueChanged(int)));
    connect(this->sui->getClusterCheckBox(), SIGNAL(stateChanged(int)), this, SLOT(on_clusterCheckBox_stateChanged(int)));*/
    connect(cd, SIGNAL(addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsConstraint(std::string, double, double, double, unsigned int, unsigned int, bool)));

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

void MainWindow::init(){

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
    this->clusterChanged = true;
    this->constrained = false;
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
    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->annotationSelectionStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    this->deformationStyle = vtkSmartPointer<MeshDeformationStyle>::New();
    this->deformationStyle->setInfoPalette(iui);
    this->soCoords = nullptr;
    this->slicer = nullptr;
    this->solver = nullptr;
    this->pointsSelectionStatus.clear();
    this->ren = vtkSmartPointer<vtkRenderer>::New();
    this->ren->SetRenderWindow(this->ui->qvtkWidget->GetRenderWindow());
    this->ren->GetRenderWindow()->Render();
    this->ren->AddActor(meshAssembly);
    this->ren->SetBackground(1.0, 1.0, 1.0);
    this->ren->SetLayer(0);
    this->sliceRen = vtkSmartPointer<vtkRenderer>::New();
    this->sliceRen->SetBackground(1.0, 1.0, 1.0);
    this->sliceRen->GetActiveCamera()->SetPosition(center.x, center.y + 2 * totalHeight, center.z);
    this->sliceRen->GetActiveCamera()->SetViewUp(0, 0, 1);
    this->sliceRen->GetActiveCamera()->SetFocalPoint(center.x, center.y, center.z);
    this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(ren);
    this->sui->getSlicerWidget()->GetRenderWindow()->AddRenderer(sliceRen);
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

    VTK_CREATE(vtkVectorText, text);
    VTK_CREATE(vtkActor, actor);
    VTK_CREATE(vtkPolyDataMapper, mapper);
    text->SetText(message.c_str());
    mapper->SetInputConnection(text->GetOutputPort());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0,0,0);
    this->meshAssembly->AddPart(actor);
    this->ren->ResetCamera();
    this->ui->qvtkWidget->update();

}

void MainWindow::updateView(){

//    VTK_CREATE(vtkPlane, intersectingPlane);
//    intersectingPlane->SetOrigin(center.x, center.y, center.z);
//    intersectingPlane->SetNormal(normal.x, normal.y, normal.z);
//    this->ren->Clear();
//    this->sliceRen->Clear();

//    if(model != nullptr)
//        this->model->draw(meshAssembly);
//    if(cage != nullptr)
//        this->cage->draw(meshAssembly);
//    this->meshAssembly->RemovePart(visualPropertiesAssembly);
//    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(visualPropertiesAssembly);
//    this->sliceRen->RemoveActor(sliceAssembly);
//    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(sliceAssembly);
//    VTK_CREATE(vtkCutter, cutter);
//    cutter->SetCutFunction(intersectingPlane);
//    VTK_CREATE(vtkPolyData, poly);
//    poly->SetPoints(this->model->getPoints());
//    poly->SetPolys(this->model->getTriangles());
//    cutter->SetInputData(poly);
//    cutter->Update();
//    VTK_CREATE(vtkPolyData, pd);
//    pd = cutter->GetOutput();
//    VTK_CREATE(vtkPolyDataMapper, cutterMapper);
//    cutterMapper->SetInputData(pd);

//    VTK_CREATE(vtkActor, sliceActor);
//    sliceActor->SetMapper(cutterMapper);
//    sliceActor->GetProperty()->SetOpacity(1);
//    sliceActor->GetProperty()->SetLineWidth(3);
//    sliceActor->GetProperty()->SetColor(0, 0.5, 0);
//    sliceActor->GetProperty()->SetRepresentationToWireframe();
//    if(this->showSliceOnMesh && this->sui->isVisible())
//        this->visualPropertiesAssembly->AddPart(sliceActor);

//    if(this->showBoundingBox){

//        if(this->boundingBox.size() == 0){
//            vector<double> bb = Utilities::getOBB(model);
//            this->boundingBox.insert(this->boundingBox.end(), bb.begin(), bb.end());
//        }

//        VTK_CREATE(vtkPolyData, boundingBoxData);
//        VTK_CREATE(vtkPolyDataMapper, boundingBoxMapper);
//        VTK_CREATE(vtkActor, boundingBoxActor);
//        VTK_CREATE(vtkOutlineSource, boundingBoxSource);

//        boundingBoxSource->SetBoxTypeToOriented();
//        boundingBoxSource->SetCorners(this->boundingBox.data());
//        boundingBoxSource->Update();
//        boundingBoxMapper->SetInputConnection(boundingBoxSource->GetOutputPort());
//        boundingBoxActor->SetMapper(boundingBoxMapper);
//        boundingBoxActor->GetProperty()->SetColor(1, 0, 0);
//        this->visualPropertiesAssembly->AddPart(boundingBoxActor);
//    }

//    if(this->showClusters){

//        uint clusterPosition = 0;
//        for(vector<vector<uint> >::iterator cit = this->clusters.begin(); cit != this->clusters.end(); cit++){
//            vector<uint> cluster = static_cast<vector<uint> >(*cit);
//            for(vector<uint>::iterator pit = cluster.begin(); pit != cluster.end(); pit++){

//                Slice* s = this->clusteringSlices[*pit];
//                vector<IMATI_STL::Point*> boundary = s->getBoundary();
//                VTK_CREATE(vtkActor, sliceBoundaryActor);
//                Utilities::renderSpline(boundary, sliceBoundaryActor);
//                vector<double> color;
//                for(uint i = 0; i < s->getHoles().size(); i++){
//                    color.clear();
//                    VTK_CREATE(vtkActor, sliceHoleActor);
//                    Utilities::renderSpline(s->getHoles()[i], sliceHoleActor);
//                    color.insert(color.end(), colors[clusterPosition].begin(), colors[clusterPosition].end());
//                    sliceHoleActor->GetProperty()->SetColor(color.data());
//                    sliceHoleActor->GetProperty()->SetLineWidth(2);
//                    this->visualPropertiesAssembly->AddPart(sliceHoleActor);
//                }
//                color.clear();
//                color.insert(color.end(), colors[clusterPosition].begin(), colors[clusterPosition].end());
//                sliceBoundaryActor->GetProperty()->SetColor(color.data());
//                sliceBoundaryActor->GetProperty()->SetLineWidth(2);
//                this->visualPropertiesAssembly->AddPart(sliceBoundaryActor);
//            }
//            clusterPosition++;
//        }
//    }

//    if(this->slicesExtracted){
//        vector<Slice*> slices = this->slicer->getSlices();

//        IMATI_STL::Point slicesCenter(0, 0, 0);
//        for(uint i = 0; i < slices.size(); i++){

//            DrawableMesh* sliceMesh = new DrawableMesh(static_cast<DrawableMesh*>(slices[i]->getSlice()));
//            sliceMesh->buildVTKStructure();
//            sliceMesh->setMeshModified(true);
//            sliceMesh->setAnnotationsModified(true);
//            sliceMesh->update();
//            slicesCenter += sliceMesh->getCenter();
//            if(this->sui->isVisible()){
//                sliceMesh->draw(this->sliceAssembly);
//                VTK_CREATE(vtkActor, sliceContourActor);
//                vector<IMATI_STL::Point*> contour = slices[i]->getBoundary();
//                Utilities::removeCreeks(contour, 10);

//                //*******************************************************************************
//                if(contour.size() > 2){
//                    vector<IMATI_STL::Point*> boundary;
//                    for(uint i = 0; i < contour.size() - 1; i++){
//                        IMATI_STL::Point* p = contour[i];
//                        int p_index = Utilities::mod(i - 1, contour.size() - 1);
//                        int n_index = Utilities::mod(i + 1, contour.size() -1);
//                        IMATI_STL::Point* prev = contour[p_index];
//                        IMATI_STL::Point* next = contour[n_index];
//                        IMATI_STL::Point* n;
//                        Eigen::Vector2d e_prev = {prev->x, prev->z};
//                        Eigen::Vector2d e_p = {p->x, p->z};
//                        Eigen::Vector2d e_next = {next->x, next->z};
//                        IMATI_STL::Point v1 = (*prev - *p);
//                        IMATI_STL::Point v2 = (*next - *p);
//                        v1.normalize();
//                        v2.normalize();

//                        n = new IMATI_STL::Point(v1 + v2);
//                        if(Utilities::isLeft(e_next, e_prev, e_p))
//                            *n *= -1;

//                        n->normalize();
//                        IMATI_STL::Point* tmp = new IMATI_STL::Point(*contour[i] + (*n) * 0.2);
//                        boundary.push_back(tmp);
//                    }
//                    boundary.push_back(boundary[0]);
//                    //*******************************************************************************
//                    //Utilities::renderSpline(contour, sliceContourActor);
//                    sliceContourActor->GetProperty()->SetColor(0, 0.5, 0);
//                    sliceContourActor->GetProperty()->SetLineWidth(1);
//                    sliceAssembly->AddPart(sliceContourActor);
//                    for(uint j = 0; j < slices[i]->getHoles().size(); j++){
//                        VTK_CREATE(vtkActor, sliceHoleContourActor);
//                        Utilities::renderSpline(slices[i]->getHoles()[j], sliceHoleContourActor);
//                        sliceHoleContourActor->GetProperty()->SetColor(0, 0.5, 0);
//                        sliceHoleContourActor->GetProperty()->SetLineWidth(3);
//                        sliceAssembly->AddPart(sliceHoleContourActor);
//                    }
//                }
//            }

//            if(showFittedCircle){
//                vector<pair<IMATI_STL::Point*, double> > circles = slices[i]->getBestFittedCircles();

//                for(uint j = 0; j < circles.size(); j++){
//                    VTK_CREATE(vtkRegularPolygonSource, circleSource);
//                    circleSource->GeneratePolygonOff();
//                    circleSource->SetNumberOfSides(50);
//                    circleSource->SetNormal(0, 1, 0);
//                    circleSource->SetRadius(circles[j].second);
//                    circleSource->SetCenter(circles[j].first->x, circles[j].first->y, circles[j].first->z);
//                    VTK_CREATE(vtkPolyDataMapper, circleMapper);
//                    circleMapper->SetInputConnection(circleSource->GetOutputPort());
//                    VTK_CREATE(vtkActor, circleActor);
//                    circleActor->SetMapper(circleMapper);
//                    vector<double> color = {BORDEAUX[0], BORDEAUX[1], BORDEAUX[2]};
//                    circleActor->GetProperty()->SetColor(color.data());
//                    circleActor->GetProperty()->SetLineWidth(3);
//                    sliceAssembly->AddPart(circleActor);
//                }

//            }

//            if(showCenters){

//                VTK_CREATE(vtkPolyData, tmpData);
//                VTK_CREATE(vtkPolyData, centersData);
//                VTK_CREATE(vtkPoints, centersPoints);
//                VTK_CREATE(vtkVertexGlyphFilter, centersFilter);
//                IMATI_STL::Point c = slices[i]->getSlice()->getCenter();
//                if(showFittedCircle){
//                    vector<pair<IMATI_STL::Point*, double> > circles = slices[i]->getBestFittedCircles();
//                    pair<IMATI_STL::Point*, double> circle = circles[0];
//                    centersPoints->InsertNextPoint(circle.first->x, circle.first->y, circle.first->z);
//                }
//                centersPoints->InsertNextPoint(c.x, c.y, c.z);
//                tmpData->SetPoints(centersPoints);
//                centersFilter->SetInputData(tmpData);
//                centersFilter->Update();
//                centersData->ShallowCopy(centersFilter->GetOutput());
//                VTK_CREATE(vtkDoubleArray, colors);
//                VTK_CREATE(vtkPolyDataMapper, centersMapper);
//                centersMapper->SetInputData(centersData);
//                VTK_CREATE(vtkActor, centersActor);
//                centersActor->SetMapper(centersMapper);
//                centersActor->GetProperty()->SetPointSize(3);
//                centersActor->GetProperty()->SetColor(1, 0, 0);
//                sliceAssembly->AddPart(centersActor);

//            }

//            if(showConvexHull){

//                slices[i]->computeConvexHull();
//                vector<IMATI_STL::Point*> convexHull = slices[i]->getConvexHull();
//                VTK_CREATE(vtkActor, chActor);
//                Utilities::renderSpline(convexHull, chActor);
//                chActor->GetProperty()->SetColor(1,0,1);
//                sliceAssembly->AddPart(chActor);

//            }

//            if(showBoundingRectangle){

//                slices[i]->computeBoundingBox();
//                vector<IMATI_STL::Point*> bb = slices[i]->getBoundingBox();
//                bb.push_back(bb[0]);
//                VTK_CREATE(vtkActor, boundingRectangleActor);
//                Utilities::renderSpline(bb, boundingRectangleActor);
//                boundingRectangleActor->GetProperty()->SetColor(1, 0, 1);
//                sliceAssembly->AddPart(boundingRectangleActor);

//            }

//            if(showDiagonals){

//                slices[i]->computeBoundingBox();
//                VTK_CREATE(vtkPolyData, diagonalsData);
//                VTK_CREATE(vtkPoints, diagonalsPoints);
//                VTK_CREATE(vtkCellArray, diagonalsLines);

//                pair<IMATI_STL::Point, IMATI_STL::Point> minDiagonal = slices[i]->getMinDiagonal();
//                pair<IMATI_STL::Point, IMATI_STL::Point> maxDiagonal = slices[i]->getMaxDiagonal();

//                diagonalsPoints->InsertNextPoint(minDiagonal.first.x, minDiagonal.first.y, minDiagonal.first.z);
//                diagonalsPoints->InsertNextPoint(minDiagonal.second.x, minDiagonal.second.y, minDiagonal.second.z);
//                diagonalsPoints->InsertNextPoint(maxDiagonal.first.x, maxDiagonal.first.y, maxDiagonal.first.z);
//                diagonalsPoints->InsertNextPoint(maxDiagonal.second.x, maxDiagonal.second.y, maxDiagonal.second.z);
//                VTK_CREATE(vtkLine, diagonal1);
//                diagonal1->GetPointIds()->SetId(0, 0);
//                diagonal1->GetPointIds()->SetId(1, 1);
//                diagonalsLines->InsertNextCell(diagonal1);
//                VTK_CREATE(vtkLine, diagonal2);
//                diagonal2->GetPointIds()->SetId(0, 2);
//                diagonal2->GetPointIds()->SetId(1, 3);
//                diagonalsLines->InsertNextCell(diagonal2);


//                diagonalsData->SetPoints(diagonalsPoints);
//                diagonalsData->SetLines(diagonalsLines);

//                VTK_CREATE(vtkPolyDataMapper, diagonalsMapper);
//                diagonalsMapper->SetInputData(diagonalsData);
//                VTK_CREATE(vtkActor, diagonalsActor);
//                diagonalsActor->SetMapper(diagonalsMapper);
//                diagonalsActor->GetProperty()->SetColor(1,0,1);
//                sliceAssembly->AddPart(diagonalsActor);

//            }

//            if(showSkeleton){

//                AndreasStructures::MANode* skeleton = new AndreasStructures::MANode(slices[i]->getSkeleton());

//                map<Vertex*, double> pointsCurvature = Utilities::computePolygonCurvature(slices[i]->getSlice());
//                skeleton = Utilities::simplifySkeleton(skeleton, levelOfDetail);

//                vector<AndreasStructures::MAArcPath*> arcs = skeleton->getGraphArcs();
//                VTK_CREATE(vtkPolyData, skeletonData);
//                VTK_CREATE(vtkPolyData, skeletonNodesData);

//                VTK_CREATE(vtkPoints, skeletonPoints);
//                VTK_CREATE(vtkPoints, skeletonNodesPoints);
//                VTK_CREATE(vtkCellArray, skeletonLines);

//                VTK_CREATE(vtkIdList, cellpoints);
//                for(vector<AndreasStructures::MAArcPath*>::iterator ait = arcs.begin(); ait != arcs.end(); ait++){
//                    AndreasStructures::MAArcPath* p = static_cast<AndreasStructures::MAArcPath*>(*ait);
//                    VTK_CREATE(vtkLine, line);
//                    Vertex* p1 = p->getN1()->getPoint();
//                    Vertex* p2 = p->getN2()->getPoint();
//                    vtkIdType pid1, pid2;

//                    pid1 = skeletonPoints->InsertNextPoint(p1->x, p1->y, p1->z);
//                    cellpoints->InsertNextId(pid1);

//                    vector<IMATI_STL::Point*> commonPath = p->getPath();

//                    for(vector<IMATI_STL::Point*>::iterator pit = commonPath.begin(); pit != commonPath.end(); pit++){
//                        pid2 = skeletonPoints->InsertNextPoint((*pit)->x, (*pit)->y, (*pit)->z);
//                        VTK_CREATE(vtkLine, line);
//                        line->GetPointIds()->SetId(0, pid1);
//                        line->GetPointIds()->SetId(1, pid2);
//                        skeletonLines->InsertNextCell(line);
//                        pid1 = pid2;
//                    }

//                    pid2 = skeletonPoints->InsertNextPoint(p2->x, p2->y, p2->z);
//                    cellpoints->InsertNextId(pid2);


//                    VTK_CREATE(vtkLine, lastLine);
//                    line->GetPointIds()->SetId(0, pid1);
//                    line->GetPointIds()->SetId(1, pid2);
//                    skeletonLines->InsertNextCell(line);
//                }

//                for(int i = 0; i < cellpoints->GetNumberOfIds(); i++){
//                    vtkIdType id = cellpoints->GetId(i);
//                    skeletonNodesPoints->InsertNextPoint(skeletonPoints->GetPoint(id));
//                }
//                skeletonNodesData->SetPoints(skeletonNodesPoints);
//                VTK_CREATE(vtkVertexGlyphFilter, filter1);
//                filter1->SetInputData(skeletonNodesData);
//                filter1->Update();
//                VTK_CREATE(vtkVertexGlyphFilter, filter2);
//                filter2->SetInputData(skeletonData);
//                filter2->Update();

//                skeletonData->SetPoints(skeletonPoints);
//                skeletonData->SetLines(skeletonLines);
//                VTK_CREATE(vtkPolyDataMapper, skeletonMapper);
//                VTK_CREATE(vtkActor, skeletonActor);
//                skeletonMapper->SetInputData(skeletonData);
//                skeletonActor->SetMapper(skeletonMapper);
//                skeletonActor->GetProperty()->SetLineWidth(2);
//                sliceAssembly->AddPart(skeletonActor);
//                skeletonActor->GetProperty()->SetColor(0, 0, 0);

//                if(showArcNodes){
//                    VTK_CREATE(vtkPolyDataMapper, skeletonPointsMapper);
//                    VTK_CREATE(vtkActor, skeletonPointsActor);
//                    skeletonPointsMapper->SetInputConnection(filter2->GetOutputPort());
//                    skeletonPointsActor->SetMapper(skeletonPointsMapper);
//                    skeletonPointsActor->GetProperty()->SetColor(255, 0, 0);
//                    skeletonPointsActor->GetProperty()->SetRenderPointsAsSpheres(true);
//                    skeletonPointsActor->GetProperty()->SetPointSize(5);
//                    sliceAssembly->AddPart(skeletonPointsActor);
//                }

//                if(showNodes){
//                    VTK_CREATE(vtkPolyDataMapper, skeletonNodesMapper);
//                    VTK_CREATE(vtkActor, skeletonNodesActor);
//                    skeletonNodesMapper->SetInputConnection(filter1->GetOutputPort());
//                    skeletonNodesActor->SetMapper(skeletonNodesMapper);
//                    skeletonNodesActor->GetProperty()->SetColor(0, 0, 255);
//                    skeletonNodesActor->GetProperty()->SetPointSize(5);
//                    skeletonNodesActor->GetProperty()->SetRenderPointsAsSpheres(true);
//                    sliceAssembly->AddPart(skeletonNodesActor);
//                }
//            }

//            delete(sliceMesh);
//        }

//        slicesCenter /= slices.size();
//        visualPropertiesAssembly->Modified();
//        meshAssembly->AddPart(visualPropertiesAssembly);
//        sliceRen->AddActor(sliceAssembly);
//        double f[3] = {slicesCenter.x, slicesCenter.y, slicesCenter.z};
//        sliceRen->GetActiveCamera()->SetFocalPoint(f);
//        sliceRen->GetActiveCamera()->SetPosition(f[0], f[1] + 100, f[2]);
//        sliceRen->Modified();
//        this->sui->getSlicerWidget()->update();
//    }

//    this->ui->qvtkWidget->update();

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
        slicer = new Slicer();
        slicer->setCenter(center);
        slicer->setNormal(normal);
        slicer->setMesh(model);
        slicer->setMaxSimplificationError(allowedError);
        slicer->setTransformationMatrix(transformationMatrix);
        slicer->slice();

        vector<Slice*> slices = slicer->getSlices();
        for(uint i = 0; i < static_cast<uint>(slices.size()); i++){
            AndreasStructures::MANode* skeleton = Utilities::medialAxisTransform(slices[i]->getSlice());
            slices[i]->setSkeleton(skeleton);
        }
        slicer->setSlices(slices);
        this->slicesExtracted = true;

        updateView();
        this->ui->qvtkWidget->update();
    }

}

void MainWindow::clear(){

    this->setWindowTitle("MainWindow");
    this->meshAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(this->meshAssembly);
    this->sliceAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(sliceAssembly);
    this->newWindow = true;
    this->cageLoaded = false;
    this->showClusters = false;
    this->clusteringDone = false;
    this->slicesExtracted = false;
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
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::NewInstance(modelTriangles);
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->annotationSelectionStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    this->sui->getSlicerWidget()->GetRenderWindow()->RemoveRenderer(sliceRen);
    meshAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(meshAssembly);
    visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(visualPropertiesAssembly);
    sliceAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(sliceAssembly);
    sliceRen = vtkSmartPointer<vtkRenderer>::NewInstance(sliceRen);
    sliceRen->AddActor(sliceAssembly);
    sliceRen->SetBackground(1.0,1.0,1.0);
    this->resetCamera();
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->sui->setVisible(false);
    this->sui->getSlicerWidget()->GetRenderWindow()->AddRenderer(sliceRen);
    this->sui->getClusterCheckBox()->setChecked(false);
    this->ui->actionVerticesSelection->setEnabled(false);
    this->ui->actionVerticesDeselection->setEnabled(false);
    this->ui->actionMeshDeformation->setEnabled(false);
    this->ui->actionStretch->setEnabled(false);
    this->sui->getTabWidget()->setEnabled(false);
    this->ui->actionAddVerticesRectangle->setEnabled(false);
    this->ui->actionAddAnnotationsConstraint->setEnabled(false);
    this->ui->actionConstrain->setEnabled(false);
    this->ui->actionSelectAnnotations->setEnabled(true);
    this->ui->actionSelectAnnotations->setChecked(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    VTK_CREATE(vtkInteractorStyleImage, imageStyle);
    this->sui->getSlicerWidget()->GetRenderWindow()->GetInteractor()->SetInteractorStyle(imageStyle);
    //this->clearAll();

}

void MainWindow::slotOpenFile(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           "./../../",
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();

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
                           "./../../",
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        QFileInfo info(filename);
        modelFilename = info.fileName();
        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();
        this->model = new DrawableMesh();

        if(this->model->load(filename.toStdString().c_str()) != 0)
            this->write("Can't open mesh file");
        else
            this->meshLoadedPreparation();

    }

}

void MainWindow::slotOpenCage(){

    if(!cageLoaded){

        QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D cage",
                           "./../../",
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

        if (!filename.isEmpty()){

            QFileInfo info(filename);
            cageFilename = info.fileName();
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
                VTK_CREATE(vtkIdFilter, idFilterCage);
                idFilterCage->SetInputData(cagePoints);
                idFilterCage->PointIdsOn();
                idFilterCage->SetIdsArrayName("OriginalIds");
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
                       "./../../",
                       "Coords(*.coord);; All(*.*)");

    if (!filename.isEmpty()){
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
        this->ui->actionConstrain->setEnabled(true);
    }
}

void MainWindow::slotOpenConstraints()
{
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Load the constraints on the model",
                       ".",
                       "CSTR(*.cstr);;");


    if (!filename.isEmpty()){
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
                           "./../../",
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        DrawableMesh* newFragment = new DrawableMesh();

        if(newFragment->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{
            QFileInfo info(filename);
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
            visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(visualPropertiesAssembly);
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
        model->save(filename.toStdString().c_str(),0);

}

void MainWindow::slotSaveCage(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the cage",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty())
        cage->save(filename.toStdString().c_str(),0);

}

void MainWindow::slotSaveBC(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the Barycentric Coordinates",
                       ".",
                       "COORD(*.coord);;");

    if (!filename.isEmpty())
        soCoords->saveCoordinates(filename.toStdString().c_str());
}

void MainWindow::slotSaveConstraints()
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the constraints on the mesh",
                       ".",
                       "CSTR(*.cstr);;");

    if (!filename.isEmpty()){
        ConstraintsFileManager manager;
        manager.setMesh(model);
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
    this->annotationSelectionStyle = vtkSmartPointer<AnnotationSelectionInteractorStyle>::New();
    this->cagePoints->SetPoints(this->cage->getPoints());
    VTK_CREATE(vtkIdFilter, idFilter);
    idFilter->SetInputData(cagePoints);
    idFilter->PointIdsOn();
    idFilter->SetIdsArrayName("OriginalIds");
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
    this->ui->actionConstrain->setEnabled(true);
    this->deformationStyle->setSoCoords(soCoords);
    this->deformationStyle->setCoordsComputed(true);

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
    this->iui->setVisible(!value);
    this->lui->setVisible(!value);
    this->ui->actionInfoPalette->setChecked(!value);
    updateView();
    if(value)
        updateIntersection();
}

void MainWindow::slotInfoPalette(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->iui->setVisible(value);
    this->lui->setVisible(!value);
    this->sui->setVisible(!value);
    this->ui->actionSlicerPalette->setChecked(!value);
    this->ui->actionLayerDialog->setChecked(!value);
    updateView();
}

void MainWindow::slotLayerDialog(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->iui->setVisible(!value);
    this->lui->setVisible(value);
    this->sui->setVisible(!value);
    this->ui->actionSlicerPalette->setChecked(!value);
    this->ui->actionInfoPalette->setChecked(!value);
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

void MainWindow::slotSelectAnnotations(bool checked)
{
    if (checked) {
        iren->SetInteractorStyle(annotationSelectionStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionCamera->setChecked(false);
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

        for(uint i = 0; i < cS.size(); i++){
            for(uint j = i; j < cS.size(); j++){
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

        for(uint i = 0; i < cS.size(); i++){
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
    clearAnnotations();
    clearSelections();
    clearConstraints();
    clearCoordinates();
    clearSecondaryMeshes();
    clearCamera();
    vtkSmartPointer<vtkAreaPicker> picker = vtkSmartPointer<vtkAreaPicker>::New();
    this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(picker);
}

void MainWindow::clearMesh()
{
    this->meshLoaded = false;
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
    this->ui->actionAddAnnotationsConstraint->setEnabled(false);
    this->ui->actionSelectAnnotations->setEnabled(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->qvtkWidget->update();
    clearSelections();
    clearAnnotations();
    clearCoordinates();
}

void MainWindow::clearCage()
{
    this->cageLoaded = false;
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
    this->lui->setBcComputed(false);
}

void MainWindow::clearSelections()
{
    this->ui->qvtkWidget->update();
}

void MainWindow::clearAnnotations()
{
    vector<Annotation*> empty_vector;
    if(model != nullptr){
        this->model->setAnnotations(empty_vector);
        model->setAnnotationsModified(true);
        model->update();
        model->draw(meshAssembly);
    }
    this->ui->actionAnnotation->setChecked(false);
    this->ui->qvtkWidget->update();
    this->constraints.clear();
    this->constrained = false;
    this->deformationStyle->setConstrained(false);
    vector<shared_ptr<ShapeOp::Constraint> > c;
    this->deformationStyle->setConstraints(c);
}

void MainWindow::clearAnnotations(DrawableMesh *mesh)
{
    if(mesh != nullptr){
        mesh->clearAnnotations();
        mesh->setAnnotationsModified(true);
        mesh->update();
        mesh->draw(meshAssembly);
    }
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

void MainWindow::slotClearAll(){

    clearAll();
    init();
    write(this->initialInstruction);

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

        VTK_CREATE(vtkInteractorStyleTrackballCamera, cameraStyle);
        iren->SetInteractorStyle(cameraStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionSelectAnnotations->setChecked(false);

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
        this->ui->actionSelectAnnotations->setChecked(false);

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
        this->ui->actionSelectAnnotations->setChecked(false);

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
        this->ui->actionSelectAnnotations->setChecked(false);
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
        this->ui->actionSelectAnnotations->setChecked(false);
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

    this->annotationSelectionStyle->setMesh(this->model);
    this->annotationSelectionStyle->setRen(this->ren);
    this->annotationSelectionStyle->setAssembly(this->meshAssembly);
    this->annotationSelectionStyle->setQvtkWidget(this->ui->qvtkWidget);
    this->annotationSelectionStyle->resetSelection();

    VTK_CREATE(vtkIdFilter, idFilter);
    idFilter->SetInputData(modelTriangles);
    idFilter->PointIdsOn();
    idFilter->SetIdsArrayName("OriginalIds");
    idFilter->Update();

    VTK_CREATE(vtkIdFilter, idFilterMesh);
    idFilterMesh->SetInputData(modelPoints);
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetIdsArrayName("OriginalMeshIds");
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
    this->ui->actionAddAnnotationsConstraint->setEnabled(true);
    this->ui->actionSelectAnnotations->setEnabled(true);
    this->ui->actionSelectAnnotations->setChecked(false);
    this->ui->actionShowMesh->setEnabled(true);
    this->ui->actionShowMesh->setChecked(true);
    this->ui->actionSlicerPalette->setEnabled(true);
    this->ui->actionInfoPalette->setEnabled(true);
    this->ui->verticesLabel->setText("Vertices: " + QString::number(model->V.numels()));
    this->ui->edgesLabel->setText("Edges: " + QString::number(model->E.numels()));
    this->ui->trianglesLabel->setText("Triangles: " + QString::number(model->T.numels()));
    this->meshAssembly->RemovePart(visualPropertiesAssembly);
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(visualPropertiesAssembly);
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

void MainWindow::slotActionAddAnnotationsConstraint()
{
    vector<DrawableAnnotation*> meshAnnotations = model->getDAnnotations();
    vector<Annotation*> selected = annotationSelectionStyle->getSelectedAnnotations();
    cd->setSubjects(selected);
    cd->show();
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
    this->laplacianConstraintsID.clear();

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
        vector<int> vvids = {vid};
        for (IMATI_STL::Node* n1 = v->VV()->head(); n1 != nullptr; n1 = n1->next())
            vvids.push_back(static_cast<int>(model->getPointId(static_cast<Vertex*>(n1->data))));
        auto l = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", vvids, 2 * weight, modelPointsTransposed);
        this->constraints.push_back(l);
        laplacianConstraintsID.push_back(static_cast<unsigned int>(solver->addConstraint(l)));
    }

    this->deformationStyle->setClosenessConstraintsID(closenessConstraintsID);
    this->deformationStyle->setLaplacianConstraintsID(laplacianConstraintsID);

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
}

void MainWindow::slotTransfer(){

    DrawableMesh* otherModel = new DrawableMesh();
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Choose a 3D model",
                       "./../../",
                       "STL(*.stl);;OBJ(*.obj);;PLY(*.ply);;All(*.*)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;

        if(otherModel->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{

            this->ren->RemoveActor(meshAssembly);
            meshAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(meshAssembly);
            visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::NewInstance(visualPropertiesAssembly);
            std::vector<Annotation*> annotations = this->model->getAnnotations();
            for(std::vector<Annotation*>::iterator ait = annotations.begin(); ait != annotations.end(); ait++){
                Annotation* annotation = static_cast<Annotation*>(*ait);
                Annotation* otherAnnotation;
                if(this->model->V.numels() > otherModel->V.numels() / 10)
                    otherAnnotation = annotation->transfer(otherModel, Utilities::EUCLIDEAN_DISTANCE);
                else
                    otherAnnotation = annotation->parallelTransfer(otherModel, Utilities::EUCLIDEAN_DISTANCE);
                otherAnnotation->setId(annotation->getId());
                otherAnnotation->setTag(annotation->getTag());
                otherAnnotation->setMesh(otherModel);
                otherAnnotation->setHierarchyLevel(annotation->getHierarchyLevel());
                otherModel->addAnnotation(otherAnnotation);
            }

            otherModel->setAnnotationsModified(true);
            otherModel->update();

            slotClearMesh();
            model = otherModel;
            modelFilename = filename;
            modelCanvas = model->getCanvas();
            this->meshLoadedPreparation();
            this->ui->verticesLabel->setText(QString::fromStdString("Vertices: ") + QString::number(model->V.numels()));
            this->ui->edgesLabel->setText(QString::fromStdString("Edges: ") + QString::number(model->E.numels()));
            this->ui->trianglesLabel->setText(QString::fromStdString("Triangles: ") + QString::number(model->T.numels()));

            model->setMeshModified(true);
            model->setAnnotationsModified(true);
            model->update();
            model->draw(meshAssembly);
            meshAssembly->AddPart(visualPropertiesAssembly);
            resetCamera();
        }
    }

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
    AnnotationWindow* annWin = new AnnotationWindow();
    meshAssembly->RemovePart(mesh->getCanvas());
    annWin->setMesh(mesh);
    annWin->show();
    connect(annWin, SIGNAL(annotationWindowClosed(DrawableMesh*)), this, SLOT(slotAnnotationWindowClosed(DrawableMesh*)));
    connect(annWin, SIGNAL(constrainRelationship(AnnotationsRelationship*&)), this, SLOT(slotConstrainRelationship(AnnotationsRelationship*&)));
}

void MainWindow::slotAdaptTemplate(DrawableMesh * mesh)
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
        {
            ShapeOp::Matrix3X src, tgt;
            src.resize(3, static_cast<long>(correspondences.size()));
            tgt.resize(3, static_cast<long>(correspondences.size()));
            for(unsigned int i = 0; i < correspondences.size(); i++){
                vector<Vertex*> ps = {model->getPoint(static_cast<unsigned long>(correspondences[i].first))};
                PointAnnotation* a = new PointAnnotation();
                unsigned char color[3] = {255,0,0};
                a->setId(model->getAnnotations().size());
                a->setPoints(ps);
                a->setTag(std::to_string(i));
                a->setMesh(model);
                a->setColor(color);
                //model->addAnnotation(a);
                a = new PointAnnotation();
                ps.clear();
                ps.push_back(mesh->getPoint(static_cast<unsigned long>(correspondences[i].second)));
                a->setId(mesh->getAnnotations().size());
                a->setPoints(ps);
                a->setTag(std::to_string(i));
                a->setMesh(mesh);
                color[0] = 0;
                color[1] = 255;
                a->setColor(color);
                //mesh->addAnnotation(a);
//                model->setAnnotationsModified(true);
//                mesh->setAnnotationsModified(true);
                Vertex* srcPt = model->getPoint(static_cast<unsigned long>(correspondences[i].first));
                Vertex* tgtPt = mesh->getPoint(static_cast<unsigned long>(correspondences[i].second));
                src.col(i) = ShapeOp::Vector3(srcPt->x, srcPt->y, srcPt->z);
                tgt.col(i) = ShapeOp::Vector3(tgtPt->x, tgtPt->y, tgtPt->z);
            }
            int pos = static_cast<int>(std::find(fragments.begin(), fragments.end(), mesh) - fragments.begin());
            ShapeOp::Matrix44 transformation = Eigen::umeyama(src, tgt, true);
            ShapeOp::MatrixXX transformedModelPoints = soModelPoints->transpose();
            ShapeOp::MatrixXX transformedCagePoints = soCagePoints->transpose();
            ShapeOp::MatrixXX transformedFragmentPoints = soFragmentsPoints[static_cast<unsigned long>(pos)]->transpose();
            transformedModelPoints.conservativeResize(transformedModelPoints.rows() + 1, transformedModelPoints.cols());
            transformedModelPoints.row(transformedModelPoints.rows() - 1).setOnes();
            transformedFragmentPoints.conservativeResize(transformedFragmentPoints.rows() + 1, transformedFragmentPoints.cols());
            transformedFragmentPoints.row(transformedFragmentPoints.rows() - 1).setOnes();
            transformedCagePoints.conservativeResize(transformedCagePoints.rows() + 1, transformedCagePoints.cols());
            transformedCagePoints.row(transformedCagePoints.rows() - 1).setOnes();
            transformedModelPoints = transformation * transformedModelPoints;
            transformedFragmentPoints = transformation.inverse() * transformedFragmentPoints;
            transformedCagePoints = transformation * transformedCagePoints;
            transformedModelPoints.conservativeResize(transformedModelPoints.rows() - 1, transformedModelPoints.cols());
            transformedFragmentPoints.conservativeResize(transformedFragmentPoints.rows() - 1, transformedFragmentPoints.cols());
            transformedCagePoints.conservativeResize(transformedCagePoints.rows() - 1, transformedCagePoints.cols());
//            *soModelPoints = transformedModelPoints.transpose();
            *(soFragmentsPoints[static_cast<unsigned long>(pos)]) = transformedFragmentPoints.transpose();
            //*soCagePoints = transformedCagePoints.transpose();
//            for(unsigned int i = 0; i < model->V.numels(); i++){
//                ShapeOp::Vector3 tmp = soModelPoints->row(i);
//                double p[3] = {tmp(0), tmp(1), tmp(2)};
//                model->setPointPosition(i, p);
//            }
            for(unsigned int i = 0; i < static_cast<unsigned int>(mesh->V.numels()); i++){
                ShapeOp::Vector3 tmp = soFragmentsPoints[static_cast<unsigned long>(pos)]->row(i);
                double p[3] = {tmp(0), tmp(1), tmp(2)};
                mesh->setPointPosition(i, p);
            }
            mesh->draw(this->meshAssembly);
            this->ui->qvtkWidget->update();

            for(unsigned int i = 0; i < static_cast<unsigned int>(cage->V.numels()); i++){
                ShapeOp::Vector3 tmp = soCagePoints->row(i);
                double p[3] = {tmp(0), tmp(1), tmp(2)};
                cage->setPointPosition(i, p);
            }
        }


//        {
//            NonRigidFitting wrapper(model, mesh);
//            wrapper.startFitting();

////            CharacterFittingWrapper wrapper;
////            wrapper.setTemplate_mesh(cage);
////            wrapper.setFragment_mesh(mesh);
////            wrapper.setCorrespondences_indices(correspondences);
////            if(!wrapper.run())
////                exit(5);
////            for(unsigned int i = 0; i < soCagePoints->rows(); i++){
////                ShapeOp::Vector3 pt = {cage->getPoint(i)->x, cage->getPoint(i)->y, cage->getPoint(i)->z};
////                soCagePoints->row(i) = pt;
////            }
////            soCoords->deform();

//        }

//        {
//            //Constraints keeping component: ShapeOp library
//            ConstraintSolver * solver = new ConstraintSolver();
//            ShapeOp::Matrix3X modelPointsTransposed = soModelPoints->transpose();
    
//            double weight = 1.0;
//            for (unsigned int i = 0; i < constraints.size(); i++) {
//                auto c = shared_ptr<ShapeOp::Constraint>(constraints[i]);
//                solver->addConstraint(c);
//            }
    
//            for (unsigned int i = 0; i < correspondences.size(); i++) {
//                std::vector<int> ids = {correspondences[i].first};
//                IMATI_STL::Vertex* targetPosition = mesh->getPoint(static_cast<unsigned long>(correspondences[i].second));
//                ShapeOp::Vector3 pos = {targetPosition->x, targetPosition->y, targetPosition->z};
//                auto c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", ids, 100 * weight, modelPointsTransposed);
//                dynamic_cast<ShapeOp::ClosenessConstraint* >(c.get())->setPosition(pos);
//                solver->addConstraint(c);
//            }
    
//            for (IMATI_STL::Node* n = model->V.head(); n != nullptr; n = n->next()) {
//                Vertex* v = static_cast<Vertex*>(n->data);
//                int vid = static_cast<int>(model->getPointId(v));
//                vector<int> vvids = {vid};
//                for (IMATI_STL::Node* n1 = v->VV()->head(); n1 != nullptr; n1 = n1->next())
//                    vvids.push_back(static_cast<int>(model->getPointId(static_cast<Vertex*>(n1->data))));
//                auto l = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", vvids, weight, modelPointsTransposed);
//                solver->addConstraint(l);
//            }
    
//            solver->setModelPointsTransposed(modelPointsTransposed);
//            std::cout<<"End constraint imposition!" << std::endl << std::flush;
//            solver->setCagePoints(soCagePoints);
//            solver->setBarycentricCoordinates(this->soCoords->getCoordinates());
//            if(!solver->initialize()){
//                std::cout << "Non symmetric or non positive-definite!" << std::endl << std::flush;
//                exit(4);
//            }
    
//            auto start = std::chrono::high_resolution_clock::now();
//            solver->solve(100);
//            auto end = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
//            std::cout<<"Solved in " << duration.count() << " seconds!" << std::endl << std::flush;
    
//            for(unsigned int i = 0; i < soCagePoints->rows(); i++){
//                double pos[3] = {(*soCagePoints)(i, 0), (*soCagePoints)(i, 1), (*soCagePoints)(i, 2)};
//                this->cage->setPointPosition(i, pos);
//            }
    
//            this->cage->setMeshModified(true);
//            this->cage->update();
//            this->cage->draw(meshAssembly);
//            this->soCoords->deform();
//            ShapeOp::Matrix3X mpt;
//            mpt.resize(3, soModelPoints->rows());
//            for(unsigned int i = 0; i < soModelPoints->rows(); i++){
//                ShapeOp::Vector3 pos = soModelPoints->row(i);
//                mpt.col(i) = pos;
//            }
//            solver->setModelPointsTransposed(mpt);
//        }
        mesh->setMeshModified(true);
        mesh->update();
        mesh->draw(meshAssembly);
        AnnotationFileManager m;
        m.setMesh(model);
        m.writeAnnotations("template.ant");
        m.setMesh(mesh);
        m.writeAnnotations("fragment.ant");
        this->model->setMeshModified(true);
        this->model->update();
        this->model->draw(meshAssembly);
        this->cage->setMeshModified(true);
        this->cage->update();
        this->cage->draw(meshAssembly);
        this->meshAssembly->Modified();
        this->ui->qvtkWidget->update();
    }
}

void MainWindow::slotAnnotationWindowClosed(DrawableMesh *mesh)
{
    int flagValue = 12;
    for(unsigned int i = 0; i < mesh->getDAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getDAnnotations()[i]->getAttributes().size(); j++)
        {
            Attribute* a = mesh->getDAnnotations()[i]->getAttributes()[j];
            dynamic_cast<DrawableAttribute*>(a)->setDrawAttribute(false);
        }

    mesh->draw(meshAssembly);
    if(coordsComputed)
    {
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
    this->selectionStyle->setModel(model);
    this->selectionStyle->setRenderer(ren);
    this->ren->Modified();
    this->ui->qvtkWidget->update();
}
