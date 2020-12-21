#include "debugtriangleselectionstyle.h"


using namespace std;
using namespace IMATI_STL;

DebugTriangleSelectionStyle::DebugTriangleSelectionStyle(){
    selectionMode = true;
    showSelectedTriangles = true;
    this->cellPicker = vtkSmartPointer<vtkCellPicker>::New();
}

void DebugTriangleSelectionStyle::OnRightButtonDown(){

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

        vector<unsigned long> selected = {static_cast<unsigned long>(pickedTriangleID)};
        defineSelection(selected);
        modifySelectedTriangles();

    }
}
void DebugTriangleSelectionStyle::OnLeftButtonDown(){

    if(this->Interactor->GetControlKey())
        this->CurrentMode = VTKISRBP_SELECT;

    vtkInteractorStyleRubberBandPick::OnLeftButtonDown();
}

void DebugTriangleSelectionStyle::OnLeftButtonUp(){

    vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

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


        vtkSmartPointer<vtkSelectVisiblePoints> selectVisiblePoints = vtkSmartPointer<vtkSelectVisiblePoints>::New();
        selectVisiblePoints->SetInputConnection(glyphFilter->GetOutputPort());
        selectVisiblePoints->SetRenderer(ren);
        selectVisiblePoints->Update();
        vtkSmartPointer<vtkPolyData> selected = selectVisiblePoints->GetOutput();

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

}
void DebugTriangleSelectionStyle::OnMouseMove(){

    vtkInteractorStyleRubberBandPick::OnMouseMove();

}
void DebugTriangleSelectionStyle::defineSelection(vector<unsigned long> selected){


    for(vector<unsigned long>::iterator it = selected.begin(); it != selected.end(); it++){
        if(!trianglesSelectionStatus->at(*it) && selectionMode){

            cout<<"Triangle id "<<*it<<" with vertices:"<<endl<<flush;
            Triangle* t = mesh->getTriangle(*it);
            Vertex* v = t->v1();
            for (unsigned int i = 0; i < 3; i++) {
                cout<<"Vertex v"<<i + 1<<" with id "<<mesh->getPointId(v)<<":("<<v->x<<","<<v->y<<","<<v->z<<")"<<endl<<flush;
                v = t->nextVertex(v);
            }
            trianglesSelectionStatus->at(*it) = true;
        }
        else if(trianglesSelectionStatus->at(*it) && !selectionMode )
            trianglesSelectionStatus->at(*it) = false;
    }

}

void DebugTriangleSelectionStyle::modifySelectedTriangles(){

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

}

void DebugTriangleSelectionStyle::SetTriangles(vtkSmartPointer<vtkPolyData> triangles) {this->Triangles = triangles;}

void DebugTriangleSelectionStyle::resetSelection(){
    this->trianglesSelectionStatus->clear();

    for(IMATI_STL::Node* n = mesh->T.head(); n != nullptr; n = n->next()){
        IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
        trianglesSelectionStatus->insert(std::make_pair(mesh->getTriangleId(t), false));
    }

    modifySelectedTriangles();
}

DrawableMesh *DebugTriangleSelectionStyle::getMesh() const{
    return mesh;
}

void DebugTriangleSelectionStyle::setMesh(DrawableMesh *value){
    mesh = value;
}

bool DebugTriangleSelectionStyle::getShowSelectedTriangles() const{
    return showSelectedTriangles;
}

void DebugTriangleSelectionStyle::setShowSelectedTriangles(bool value){
    showSelectedTriangles = value;
}

vtkSmartPointer<vtkRenderer> DebugTriangleSelectionStyle::getRen() const{
    return ren;
}

void DebugTriangleSelectionStyle::setRen(const vtkSmartPointer<vtkRenderer> &value){
    ren = value;
}

vtkSmartPointer<vtkPropAssembly> DebugTriangleSelectionStyle::getAssembly() const{
    return assembly;
}

void DebugTriangleSelectionStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly> &value){
    assembly = value;
}

std::map<unsigned long, bool> *DebugTriangleSelectionStyle::getTrianglesSelectionStatus() const
{
    return trianglesSelectionStatus;
}

void DebugTriangleSelectionStyle::setTrianglesSelectionStatus(std::map<unsigned long, bool> *value)
{
    trianglesSelectionStatus = value;
}

bool DebugTriangleSelectionStyle::getSelectionMode() const
{
    return selectionMode;
}

void DebugTriangleSelectionStyle::setSelectionMode(bool value)
{
    selectionMode = value;
}
