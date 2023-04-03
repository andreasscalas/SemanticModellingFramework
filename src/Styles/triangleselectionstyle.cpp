#include "triangleselectionstyle.h"

#include <imatistl.h>
#include <utilities.h>
#include <annotationutilities.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricSpline.h>
#include <vtkSphereSource.h>
#include <vtkImplicitFunction.h>
#include <vtkPlanes.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>

using namespace std;
using namespace IMATI_STL;

TriangleSelectionStyle::TriangleSelectionStyle(){
    selectionMode = true;
    selectionType = RECTANGLE_AREA;
    visibleTrianglesOnly = true;
    lasso_started = false;
    alreadyStarted = false;
    showSelectedTriangles = true;
    firstVertex = nullptr;
    lastVertex = nullptr;
    this->annotation = new DrawableAreaAnnotation();
    splinePoints = vtkSmartPointer<vtkPoints>::New();
    SplineActor  = vtkSmartPointer<vtkActor>::New();
    SplineActor->GetProperty()->SetColor(1.0,0,0);
    SplineActor->GetProperty()->SetLineWidth(3.0);
    sphereAssembly = vtkSmartPointer<vtkPropAssembly>::New();          //Assembly of actors
    this->cellPicker = vtkSmartPointer<vtkCellPicker>::New();
}

void TriangleSelectionStyle::OnRightButtonDown(){

    //If the user is trying to pick a point...
    //The click position of the mouse is takenannotatedTriangles
    int x, y;
    x = this->Interactor->GetEventPosition()[0];
    y = this->Interactor->GetEventPosition()[1];
    this->FindPokedRenderer(x, y);
    //Some tolerance is set for the picking
    this->cellPicker->SetTolerance(0);
    this->cellPicker->Pick(x, y, 0, ren);
    //If some point has been picked...
    vtkIdType pickedTriangleID = this->cellPicker->GetCellId();
    if(pickedTriangleID > 0 && pickedTriangleID < this->mesh->T.numels()){

        vector<unsigned long> selected;
        if(lasso_started){
            Triangle* t = mesh->getTriangle(static_cast<unsigned long>(pickedTriangleID));
            this->annotation->addOutline(polygonContour);
            vector<Triangle*> innerTriangles = Utilities::regionGrowing(polygonContour, t);
            for(vector<Triangle*>::iterator tit = innerTriangles.begin(); tit != innerTriangles.end(); tit++){
                selected.push_back(mesh->getTriangleId(static_cast<Triangle*>(*tit)));
            }
            splinePoints = vtkSmartPointer<vtkPoints>::New();
            assembly->RemovePart(sphereAssembly);
            sphereAssembly = vtkSmartPointer<vtkPropAssembly>::New();
            polygonContour.clear();
            lastVertex = nullptr;
            firstVertex = nullptr;
            lasso_started = false;
            this->assembly->RemovePart(SplineActor);
        }else
            selected.push_back(static_cast<unsigned long>(pickedTriangleID));

        defineSelection(selected);
        modifySelectedTriangles();

    }
}


void TriangleSelectionStyle::OnMouseMove(){

    vtkInteractorStyleRubberBandPick::OnMouseMove();

}

void TriangleSelectionStyle::OnLeftButtonDown(){

    if(this->Interactor->GetControlKey())
        switch(selectionType){

            case RECTANGLE_AREA:

                this->CurrentMode = VTKISRBP_SELECT;
                break;

            case LASSO_AREA:

                if(!lasso_started)
                    lasso_started = true;

                else{
                    //The click position of the mouse is taken
                    int x, y;
                    x = this->Interactor->GetEventPosition()[0];
                    y = this->Interactor->GetEventPosition()[1];
                    this->FindPokedRenderer(x, y);
                    //Some tolerance is set for the picking
                    this->cellPicker->Pick(x, y, 0, ren);
                    vtkIdType cellID = this->cellPicker->GetCellId();

                    //If some point has been picked...
                    if(cellID > 0 && cellID < this->mesh->T.numels()){
                        Triangle* t = mesh->getTriangle(static_cast<unsigned long>(cellID));
                        double wc[3];
                        double bestDistance = DBL_MAX;

                        this->cellPicker->GetPickPosition(wc);
                        Vertex* p = new Vertex(wc[0], wc[1], wc[2]);
                        Vertex* v_ = t->v1();
                        Vertex* v;

                        for(int i = 0; i < 3; i++){
                            double actualDistance = ((*v_)-(*p)).length();
                            if(actualDistance < bestDistance){
                                bestDistance = actualDistance;
                                v = v_;
                            }
                            v_ = t->nextVertex(v_);
                        }

                        vtkIdType pointID = static_cast<vtkIdType>(mesh->getPointId(v));
                        vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
                        Vertex* actualVertex = mesh->getPoint(static_cast<unsigned long>(pointID));
                        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
                        sphereSource->SetCenter(actualVertex->x, actualVertex->y, actualVertex->z);
                        sphereSource->SetRadius(sphereRadius);
                        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                        mapper->SetInputConnection(sphereSource->GetOutputPort());
                        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
                        actor->GetProperty()->SetColor(0,0,1);
                        actor->SetMapper(mapper);
                        sphereAssembly->AddPart(actor);
                        assembly->AddPart(sphereAssembly);
                        if(firstVertex == nullptr){
                            firstVertex = actualVertex;
                            polygonContour.push_back(firstVertex);
                        }if(lastVertex != nullptr && lastVertex != actualVertex){
                            vector<Vertex*> newContourSegment = Utilities::dijkstra(lastVertex, actualVertex, Utilities::COMBINED_DISTANCE, false);
                            polygonContour.insert(polygonContour.end(), newContourSegment.begin(), newContourSegment.end());
                            for(unsigned int i = 0; i < newContourSegment.size(); i++)
                                splinePoints->InsertNextPoint(Triangles->GetPoint(static_cast<vtkIdType>(mesh->getPointId(newContourSegment[i]))));
                            spline->SetPoints(splinePoints);
                            vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
                            functionSource->SetParametricFunction(spline);
                            functionSource->Update();
                            ren->RemoveActor(assembly);
                            this->assembly->RemovePart(SplineActor);
                            // Setup actor and mapper
                            vtkSmartPointer<vtkPolyDataMapper> splineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            splineMapper->SetInputConnection(functionSource->GetOutputPort());
                            SplineActor->SetMapper(splineMapper);
                            SplineActor->GetProperty()->SetLineWidth(5);
                            this->assembly->AddPart(SplineActor);
                            ren->AddActor(assembly);
                        }
                        lastVertex = actualVertex;
                    }
                }

                break;

            case PAINTED_LINE:
                break;

            default: break;
        }

    vtkInteractorStyleRubberBandPick::OnLeftButtonDown();
}

void TriangleSelectionStyle::OnLeftButtonUp(){

    vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

    switch(selectionType){

        case RECTANGLE_AREA:
            if(this->CurrentMode==VTKISRBP_SELECT){
                this->CurrentMode = VTKISRBP_ORIENT;

                // Forward events

                vtkPlanes* frustum = static_cast<vtkRenderedAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

                vtkSmartPointer<vtkExtractGeometry> extractGeometry = vtkSmartPointer<vtkExtractGeometry>::New();
                extractGeometry->SetImplicitFunction(static_cast<vtkImplicitFunction*>(frustum));

                #if VTK_MAJOR_VERSION <= 5
                    extractGeometry->SetInput(this->Triangles);
                #else
                    extractGeometry->SetInputData(this->Triangles);
                #endif
                    extractGeometry->Update();

                vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
                glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
                glyphFilter->Update();

                vtkSmartPointer<vtkPolyData> selected;
                if(visibleTrianglesOnly){
                    vtkSmartPointer<vtkSelectVisiblePoints> selectVisiblePoints = vtkSmartPointer<vtkSelectVisiblePoints>::New();
                    selectVisiblePoints->SetInputConnection(glyphFilter->GetOutputPort());
                    selectVisiblePoints->SetRenderer(this->GetCurrentRenderer());
                    selectVisiblePoints->Update();
                    selected = selectVisiblePoints->GetOutput();
                }else
                    selected = glyphFilter->GetOutput();

                vector<unsigned long> newlySelected;
                vtkSmartPointer<vtkIdTypeArray> ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));
                for(vtkIdType i = 0; ids!=nullptr && i < ids->GetNumberOfTuples(); i++){
                    vtkSmartPointer<vtkIdList> tids = vtkSmartPointer<vtkIdList>::New();
                    Triangles->GetPointCells(ids->GetValue(i), tids);
                    for(vtkIdType j = 0; j < tids->GetNumberOfIds(); j++){
                        newlySelected.push_back(static_cast<unsigned long>(tids->GetId(j)));
                    }
                }

                defineSelection(newlySelected);
                modifySelectedTriangles();

            }
            break;

    }

}

void TriangleSelectionStyle::modifySelectedTriangles(){

    if(showSelectedTriangles)
        mesh->setSelectedTriangles(*trianglesSelectionStatus);
    else{
        std::map<unsigned long, bool> zeroSelectedMap;
        for(IMATI_STL::Node* n = mesh->T.head(); n != nullptr; n = n->next()){
            IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
            zeroSelectedMap.insert(std::make_pair(mesh->getTriangleId(t), false));
        }
        mesh->setSelectedTriangles(zeroSelectedMap);
    }
    ren->RemoveActor(assembly);
    this->mesh->draw(assembly);
    ren->AddActor(assembly);
    this->qvtkWidget->update();
}

void TriangleSelectionStyle::SetTriangles(vtkSmartPointer<vtkPolyData> triangles) {this->Triangles = triangles;}

void TriangleSelectionStyle::resetSelection(){
    this->trianglesSelectionStatus->clear();

    for(IMATI_STL::Node* n = mesh->T.head(); n != nullptr; n = n->next()){
        IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
        trianglesSelectionStatus->insert(std::make_pair(mesh->getTriangleId(t), false));
    }

}

void TriangleSelectionStyle::defineSelection(vector<unsigned long> selected){
    for(vector<unsigned long>::iterator it = selected.begin(); it != selected.end(); it++){
        if(!trianglesSelectionStatus->at(*it) && selectionMode)
            trianglesSelectionStatus->at(*it) = true;
        else if(trianglesSelectionStatus->at(*it) && !selectionMode )
            trianglesSelectionStatus->at(*it) = false;
    }

}

Annotation *TriangleSelectionStyle::getAnnotation() const{
    return annotation;
}

DrawableMesh *TriangleSelectionStyle::getMesh() const{
    return mesh;
}

void TriangleSelectionStyle::setMesh(DrawableMesh *value){
    mesh = value;
    cellPicker = vtkSmartPointer<vtkCellPicker>::NewInstance(cellPicker);
    cellPicker->SetPickFromList(1);
    cellPicker->AddPickList(mesh->getMeshSurfaceActor());
    this->sphereRadius = this->mesh->getMinEdgeLength();
}

std::vector<Vertex *> TriangleSelectionStyle::getPolygonContour() const{
    return polygonContour;
}

void TriangleSelectionStyle::setPolygonContour(const std::vector<Vertex *> &value){
    polygonContour = value;
}

void TriangleSelectionStyle::setInnerVertex(Vertex *value){
    innerVertex = value;
}

unsigned int TriangleSelectionStyle::getSelectionType() const{
    return selectionType;
}

void TriangleSelectionStyle::setSelectionType(unsigned int value){
    selectionType = value;
}

bool TriangleSelectionStyle::getShowSelectedTriangles() const{
    return showSelectedTriangles;
}

void TriangleSelectionStyle::setShowSelectedTriangles(bool value){
    showSelectedTriangles = value;
}

bool TriangleSelectionStyle::getVisibleTrianglesOnly() const{
    return visibleTrianglesOnly;
}

void TriangleSelectionStyle::setVisibleTrianglesOnly(bool value){
    visibleTrianglesOnly = value;
}

bool TriangleSelectionStyle::getSelectionMode() const{
    return selectionMode;
}

void TriangleSelectionStyle::setSelectionMode(bool value){
    selectionMode = value;
}

vtkSmartPointer<vtkRenderer> TriangleSelectionStyle::getRen() const{
    return ren;
}

void TriangleSelectionStyle::setRen(const vtkSmartPointer<vtkRenderer> &value){
    ren = value;
}

vtkSmartPointer<vtkPropAssembly> TriangleSelectionStyle::getAssembly() const{
    return assembly;
}

void TriangleSelectionStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly> &value){
    assembly = value;
}

Vertex* TriangleSelectionStyle::getInnerVertex() const{
    return innerVertex;
}

void TriangleSelectionStyle::finalizeAnnotation(unsigned int id, string tag, unsigned char color[]){

    vector<IMATI_STL::Triangle*> selectedTriangles;
    for(map<unsigned long, bool>::iterator tit = trianglesSelectionStatus->begin(); tit != trianglesSelectionStatus->end(); tit++){
        pair<unsigned long, bool> p = *tit;
        if(p.second)
            selectedTriangles.push_back(mesh->getTriangle(p.first));
    }

    if(selectedTriangles.size() > 0){

        this->annotation->setId(id);
        this->annotation->setOutlines(Utilities::getOutlines(selectedTriangles));
        this->annotation->setColor(color);
        this->annotation->setTag(tag);
        this->annotation->setMeshPoints(mesh->getPoints());
        this->annotation->setMesh(mesh);
        this->mesh->addAnnotation(annotation);
        this->mesh->setAnnotationsModified(true);
        this->mesh->update();
        this->mesh->draw(assembly);
        this->resetSelection();
        this->modifySelectedTriangles();
        this->annotation = new DrawableAreaAnnotation();
    }
}



std::map<unsigned long, bool> *TriangleSelectionStyle::getTrianglesSelectionStatus() const
{
    return trianglesSelectionStatus;
}

void TriangleSelectionStyle::setTrianglesSelectionStatus(std::map<unsigned long, bool> *value)
{
    trianglesSelectionStatus = value;
}

QVTKOpenGLNativeWidget *TriangleSelectionStyle::getQvtkWidget() const
{
    return qvtkWidget;
}

void TriangleSelectionStyle::setQvtkWidget(QVTKOpenGLNativeWidget *value)
{
    qvtkWidget = value;
}
