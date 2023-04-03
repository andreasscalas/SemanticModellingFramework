#include "correspondencesselectionstyle.h"
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkLabeledDataMapper.h>
#include <vtkIdFilter.h>
#include <vtkCamera.h>

CorrespondencesSelectionStyle::CorrespondencesSelectionStyle()
{
    this->selectedActor = vtkSmartPointer<vtkActor>::New();
    this->labelsActor = vtkSmartPointer<vtkActor2D>::New();
    this->selectedPoints = vtkSmartPointer<vtkCellArray>::New();
    this->cellPicker = vtkSmartPointer<vtkCellPicker>::New();
}

CorrespondencesSelectionStyle::~CorrespondencesSelectionStyle()
{
    mesh = nullptr;
    qvtkWidget = nullptr;
    pickedVertices.clear();
}

void CorrespondencesSelectionStyle::OnLeftButtonDown()
{

    if(this->Interactor->GetControlKey()){

        int x, y;
        x = this->Interactor->GetEventPosition()[0];
        y = this->Interactor->GetEventPosition()[1];
        this->cellPicker->AddPickList(mesh->getMeshSurfaceActor());
        this->cellPicker->PickFromListOn();
        this->FindPokedRenderer(x, y);
        //Some tolerance is set for the picking
        this->cellPicker->SetTolerance(0);
        this->cellPicker->Pick(x, y, 0, ren);
        vtkIdType cellID = this->cellPicker->GetCellId();

        //If some point has been picked...
        if(cellID > 0 && cellID < this->mesh->T.numels()){

            IMATI_STL::Triangle* t = mesh->getTriangle(static_cast<unsigned long>(cellID));
            double wc[3];
            double bestDistance = DBL_MAX;

            this->cellPicker->GetPickPosition(wc);
            IMATI_STL::Vertex* p = new IMATI_STL::Vertex(wc[0], wc[1], wc[2]);
            IMATI_STL::Vertex* tv1 = t->v1(), *tv2 = t->v2(), *tv3 = t->v3();
            double* normal = this->GetCurrentRenderer()->GetActiveCamera()->GetViewPlaneNormal();
            IMATI_STL::Point n(normal[0], normal[1], normal[2]);
            IMATI_STL::Point p_ = IMATI_STL::Point::linePlaneIntersection(*p, (*p) + n, *tv1, *tv2, *tv3);
            IMATI_STL::Vertex* v_ = t->v1();
            IMATI_STL::Vertex* v;

            for(int i = 0; i < 3; i++){
                double actualDistance = ((p_)-(*v_)).length();
                if(actualDistance < bestDistance){
                    bestDistance = actualDistance;
                    v = v_;
                }
                v_ = t->nextVertex(v_);
            }

            vtkIdType pointID = static_cast<vtkIdType>(mesh->getPointId(v));
            std::vector<int>::iterator it = std::find(pickedVertices.begin(), pickedVertices.end(), pointID);
            if(it == pickedVertices.end())
                pickedVertices.push_back(pointID);
            else
                pickedVertices.erase(it);

            selectedPoints = vtkSmartPointer<vtkCellArray>::NewInstance(selectedPoints);
            vtkSmartPointer<vtkPoints> selectedPointsSet = vtkSmartPointer<vtkPoints>::New();
            for(unsigned int i = 0; i < pickedVertices.size(); i++){
                IMATI_STL::Vertex* p = mesh->getPoint(pickedVertices[i]);
                vtkIdType ids[1] = {selectedPointsSet->InsertNextPoint(p->x, p->y, p->z)};
                selectedPoints->InsertNextCell(1, ids);
            }
            ren->RemoveActor(selectedActor);
            ren->RemoveActor(labelsActor);
            selectedActor = vtkSmartPointer<vtkActor>::New();
            labelsActor = vtkSmartPointer<vtkActor2D>::New();
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkLabeledDataMapper> labelMapper = vtkSmartPointer<vtkLabeledDataMapper>::New();
            vtkSmartPointer<vtkIdFilter> ids = vtkSmartPointer<vtkIdFilter>::New();
            this->data = vtkSmartPointer<vtkPolyData>::New();
            this->data->SetPoints(selectedPointsSet);
            this->data->SetVerts(selectedPoints);
            ids->SetInputData( this->data );
            ids->PointIdsOff();
            ids->CellIdsOff();
            ids->FieldDataOff();
            ids->Modified();
            mapper->SetInputData(data);
            labelMapper->SetInputData(data);
            labelMapper->Modified();
            labelsActor->SetMapper(labelMapper);
            selectedActor->SetMapper(mapper);
            selectedActor->GetProperty()->SetPointSize(10);
            selectedActor->GetProperty()->SetColor(0,0,0);
            ren->AddActor(selectedActor);
            ren->AddActor(labelsActor);
            qvtkWidget->update();

        }

    }
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();

}

std::vector<int> CorrespondencesSelectionStyle::getPickedVertices() const
{
    return pickedVertices;
}

void CorrespondencesSelectionStyle::setRen(const vtkSmartPointer<vtkRenderer> &value)
{
    ren = value;
}

DrawableMesh *CorrespondencesSelectionStyle::getMesh() const
{
    return mesh;
}

void CorrespondencesSelectionStyle::setMesh(DrawableMesh *value)
{
    mesh = value;
    meshPoints = mesh->getPoints();
}

void CorrespondencesSelectionStyle::setCellPicker(const vtkSmartPointer<vtkCellPicker> &value)
{
    cellPicker = value;
}

QVTKOpenGLNativeWidget *CorrespondencesSelectionStyle::getQvtkWidget() const
{
    return qvtkWidget;
}

void CorrespondencesSelectionStyle::setQvtkWidget(QVTKOpenGLNativeWidget *value)
{
    qvtkWidget = value;
}
