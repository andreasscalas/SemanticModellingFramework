#include "measurestyle.h"

#include <annotationutilities.h>
#include <utilities.h>
#include <geometricattribute.h>
#include <drawableeuclideanmeasure.h>
#include <drawablegeodesicmeasure.h>
#include <drawableboundingmeasure.h>
#include <drawableannotation.h>
#include <vtkRenderer.h>

#include <vector>

#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkStripper.h>
#include <vtkFeatureEdges.h>
#include <vtkPropPicker.h>
#include <drawableheightmeasure.h>

using namespace std;
using namespace IMATI_STL;
MeasureStyle::MeasureStyle()
{
    measureStarted = false;
    leftPressed = false;
    middlePressed = false;
    drawAttributes = false;
    measureAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    measureType = MeasureType::RULER;
    boundingOrigin = nullptr;
    boundingBegin = nullptr;
    boundingEnd = nullptr;
    onCreationAttribute = nullptr;
}

MeasureStyle::~MeasureStyle()
{
    mesh = nullptr;
    qvtkwidget = nullptr;
    if(onCreationAttribute != nullptr)
        delete onCreationAttribute;
}

DrawableMesh *MeasureStyle::getMesh() const
{
    return mesh;
}

void MeasureStyle::setMesh(DrawableMesh *value)
{
    mesh = value;
    cellPicker = vtkSmartPointer<vtkCellPicker>::NewInstance(cellPicker);
    cellPicker->SetPickFromList(1);
    cellPicker->AddPickList(mesh->getMeshSurfaceActor());
}


void MeasureStyle::manageRulerMovement(IMATI_STL::Vertex* start, IMATI_STL::Vertex* end)
{
    if(start == end)
        return;
    Point measureVector = (*end) - (*start);
    double measure = measureVector.length();
    if(measure < epsilon)
        return;
    this->measure = measure;
    if(measurePath.size() == 2)
        measurePath.pop_back();
    measurePath.push_back(end);
    dynamic_cast<GeometricAttribute*>(onCreationAttribute)->clearMeasurePointsID();
    DrawableEuclideanMeasure* eucAtt = dynamic_cast<DrawableEuclideanMeasure*>(onCreationAttribute);
    eucAtt->addMeasurePointID(mesh->getPointId(measurePath[0]));
    if(measurePath.size() > 1)
        eucAtt->addMeasurePointID(mesh->getPointId(measurePath[1]));


}

void MeasureStyle::manageTapeMovement(IMATI_STL::Vertex* start, IMATI_STL::Vertex* end)
{
    vector<Vertex*> path = Utilities::dijkstra(start, end, Utilities::EUCLIDEAN_DISTANCE, true);
    path.insert(path.begin(), start);
    measurePath.insert(measurePath.end(), path.begin(), path.end());
    Point measureVector;

    for(unsigned int i = 1; i < path.size(); i++){
        measureVector = (*path[i]) - (*path[i - 1]);
        measure += measureVector.length();
    }

    DrawableGeodesicMeasure* geoAtt = dynamic_cast<DrawableGeodesicMeasure*>(onCreationAttribute);
    geoAtt->clearMeasurePointsID();
    for(unsigned int i = 0; i < measurePath.size(); i++)
    {
        geoAtt->addMeasurePointID(mesh->getPointId(measurePath[i]));
    }
}

void MeasureStyle::manageHeightMovement(IMATI_STL::Vertex* start)
{

    DrawableHeightMeasure* heightAtt = dynamic_cast<DrawableHeightMeasure*>(onCreationAttribute);
    heightAtt->clearMeasurePointsID();
    heightAtt->addMeasurePointID(mesh->getPointId(start));
    heightAtt->setDirection(0.0,0.0,1.0);
    heightAtt->update();
    measure = ((*start) - (*heightAtt->getBase())).length();
}
std::pair<Point*, Point*> compute2OrthogonalVersors(Point v)
{
    Point* v1 = new Point();
    double value1 = rand(), value2 = rand();
    if(abs(v.x) > 1e-6)
    {
        v1->x = (-value1 * v.y - value2 * v.z) / v.x;
        v1->y = value1;
        v1->z = value2;
    } else if (abs(v.y) > 1e-6)
    {
        v1->x = value1;
        v1->y = (-value1 * v.x - value2 * v.z) / v.y;
        v1->z = value2;
    } else if (abs(v.z) > 1e-6)
    {
        v1->x = value1;
        v1->y = value2;
        v1->z = (-value1 * v.x - value2 * v.y) / v.z;
    } else
    {
        v1->x = rand();
        v1->y = value1;
        v1->z = value2;
    }

    v1->normalize();

    Point* v2 = new Point(v & (*v1));
    v2->normalize();

    return make_pair(v1, v2);
}



void MeasureStyle::manageCaliberMovement()
{
    vtkSmartPointer<vtkPlane> cuttingPlane = vtkSmartPointer<vtkPlane>::New();
    double* orientation = this->CurrentRenderer->GetActiveCamera()->GetViewPlaneNormal();
    Point direction(orientation[0], orientation[1], orientation[2]);
    direction.normalize();
    Point caliperDirection((*boundingEnd) - (*boundingBegin));
    caliperDirection.normalize();
    Point normal = direction & caliperDirection;
    normal.normalize();
    cuttingPlane->SetOrigin(boundingBegin->x, boundingBegin->y, boundingBegin->z);
    cuttingPlane->SetNormal(normal.x, normal.y, normal.z);
    vtkSmartPointer<vtkPolyData> meshData = static_cast<vtkPolyData*>(mesh->getMeshSurfaceActor()->GetMapper()->GetInput());

    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetInputData(meshData);
    cutter->SetCutFunction(cuttingPlane);
    vtkSmartPointer<vtkStripper> cutStrips = vtkSmartPointer<vtkStripper>::New();
    cutStrips->SetInputConnection(cutter->GetOutputPort());
    cutStrips->Update();
    vtkSmartPointer<vtkPolyData> cutPoly = vtkSmartPointer<vtkPolyData>::New();
    cutPoly->SetPoints(cutStrips->GetOutput()->GetPoints());
    cutPoly->SetPolys(cutStrips->GetOutput()->GetLines());
    vtkSmartPointer<vtkFeatureEdges> cutBoundary = vtkSmartPointer<vtkFeatureEdges>::New();
    cutBoundary->SetInputData(cutPoly);
    cutBoundary->Update();

    vtkSmartPointer<vtkPolyData> boundaryPolydata = cutBoundary->GetOutput();
    boundaryPolydata->GetLines()->InitTraversal();
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
    std::map<vtkIdType, Point*> points;
    vector<Point*> usefulProjected;
    Point projectedExtreme1 = caliperDirection * ((*boundingBegin) * caliperDirection);
    Point projectedExtreme2 = caliperDirection * ((*boundingEnd) * caliperDirection);
    while(boundaryPolydata->GetLines()->GetNextCell(idList))
        for(vtkIdType i = 0; i < idList->GetNumberOfIds(); i++)
        {
            double* p = boundaryPolydata->GetPoint(idList->GetId(i));
            Point* p_ = new Point(p[0],p[1],p[2]);
            Point* projected = new Point(caliperDirection * ((*p_) * caliperDirection));
            if(Utilities::isPointInSegment(projected, projectedExtreme1, projectedExtreme2))
            {
                 points[idList->GetId(i)] = p_;
                 usefulProjected.push_back(projected);
            }
        }
    if(usefulProjected.size() == 0)
        return;
    std::pair<Point*, Point*> extrema = Utilities::findExtremePoints(usefulProjected, caliperDirection);
    int pos1 = std::find(usefulProjected.begin(), usefulProjected.end(), extrema.first) - usefulProjected.begin();
    int pos2 = std::find(usefulProjected.begin(), usefulProjected.end(), extrema.second) - usefulProjected.begin();
    measurePath.clear();
    Point* extremePoint1 = std::next(points.begin(), pos1)->second;
    Point* extremePoint2 = std::next(points.begin(), pos2)->second;
    measurePath.push_back(mesh->getClosestPoint(extremePoint1));
    measurePath.push_back(mesh->getClosestPoint(extremePoint2));
    measure = ((*extremePoint2) - (*extremePoint1)).length();
}

void MeasureStyle::manageBoundingMovement()
{
    std::vector<IMATI_STL::Point*> points;
    bool selected = false;
    if(boundingOrigin != nullptr)
        delete boundingOrigin;
    boundingOrigin = new Point(0, 0, 0);
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        if(dynamic_cast<DrawableAnnotation*>(mesh->getAnnotations()[i])->getSelected())
        {
            selected = true;
            std::vector<IMATI_STL::Vertex*> involved = mesh->getAnnotations()[i]->getInvolvedVertices();
            points.insert(points.end(), involved.begin(), involved.end());
        }
    for(unsigned int i = 0; i < points.size(); i++)
        *boundingOrigin += points[i];
    *boundingOrigin /= points.size();
    if(!selected)
        return;
    Point boundingDirection((*boundingEnd) - (*boundingBegin));
    if(boundingDirection.length() == 0)
        return;
    boundingDirection.normalize();

    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> extrema = Utilities::findExtremePoints(points, boundingDirection);
    measurePath.clear();
    measurePath.push_back(static_cast<Vertex*>(extrema.first));
    measurePath.push_back(static_cast<Vertex*>(extrema.second));

    DrawableBoundingMeasure* boundAtt = dynamic_cast<DrawableBoundingMeasure*>(onCreationAttribute);
    boundAtt->clearMeasurePointsID();
    boundAtt->addMeasurePointID(mesh->getPointId(measurePath[0]));
    boundAtt->addMeasurePointID(mesh->getPointId(measurePath[1]));
    boundAtt->setOrigin(boundingOrigin);
    boundAtt->setDirection(boundingDirection);
    boundAtt->update();
}

void MeasureStyle::reset()
{
    measure = 0.0;
    measurePath.clear();
    this->last = nullptr;
    if(measureType == MeasureType::BOUNDING)
    {
        if(boundingOrigin != nullptr)
            delete boundingOrigin;
        if(boundingBegin != nullptr)
            delete boundingBegin;
        if(boundingEnd != nullptr)
            delete boundingEnd;
    }
    if(onCreationAttribute != nullptr)
        delete onCreationAttribute;

    switch (measureType) {
        case MeasureType::RULER:
        {
            onCreationAttribute = new DrawableEuclideanMeasure();
            break;
        }
        case MeasureType::TAPE:
        {
            onCreationAttribute = new DrawableGeodesicMeasure();
            break;
        }
        case MeasureType::CALIBER:
        {
            //a = new DrawableCaliberMeasure();
            break;
        }
        case MeasureType::BOUNDING:
        {
            onCreationAttribute = new DrawableBoundingMeasure();
            dynamic_cast<DrawableBoundingMeasure*>(onCreationAttribute)->setDrawPlanes(true);
            break;
        }
        case MeasureType::HEIGHT:
        {
            onCreationAttribute = new DrawableHeightMeasure();
            break;
        }
    }

    onCreationAttribute->setValue(new double(0.0));
    onCreationAttribute->setRenderer(meshRenderer);
    onCreationAttribute->setMesh(mesh);

    boundingOrigin = nullptr;
    boundingBegin = nullptr;
    boundingEnd = nullptr;
    measureStarted = false;
    leftPressed = false;
    middlePressed = false;
    meshRenderer->RemoveActor(measureAssembly);
    measureAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    cellPicker = vtkSmartPointer<vtkCellPicker>::NewInstance(cellPicker);
    cellPicker->SetPickFromList(1);
    cellPicker->AddPickList(mesh->getMeshSurfaceActor());
    updateView();

}

DrawableAttribute *MeasureStyle::finalizeAttribute(unsigned int id, string key)
{
    onCreationAttribute->setId(id);
    onCreationAttribute->setKey(key);
    DrawableAttribute* returnAttribute = onCreationAttribute;
    returnAttribute->setDrawValue(true);
    if(measureType == MeasureType::BOUNDING)
        dynamic_cast<DrawableBoundingMeasure*>(returnAttribute)->setDrawPlanes(false);
    onCreationAttribute = nullptr;
    reset();
    return returnAttribute;
}

void MeasureStyle::updateView()
{
    meshRenderer->RemoveActor(measureAssembly);
    if(drawAttributes)
    {
        onCreationAttribute->setRenderer(meshRenderer);
        onCreationAttribute->draw(measureAssembly);
    }

    measureAssembly->Modified();
    meshRenderer->AddActor(measureAssembly);
    qvtkwidget->update();

}

void MeasureStyle::OnMouseMove()
{

    if(this->Interactor->GetControlKey() && measureStarted && (measureType == MeasureType::CALIBER || measureType == MeasureType::BOUNDING))
    {
        int x, y;
        x = this->Interactor->GetEventPosition()[0];
        y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);
        vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
        coord->SetCoordinateSystemToDisplay();
        coord->SetValue(x,y,0);
        double* worldCoord = coord->GetComputedWorldValue(meshRenderer);
        if(boundingEnd != nullptr)
            delete boundingEnd;
        boundingEnd = new Point(worldCoord[0], worldCoord[1], worldCoord[2]);
        switch (measureType) {
            case MeasureType::CALIBER:
            {
                manageCaliberMovement();
                break;
            }
            case MeasureType::BOUNDING:
            {
                manageBoundingMovement();
                break;
            }
            default:
                exit(4343); //IMPOSSIBLE
        }
        onCreationAttribute->update();
    }
    if(leftPressed || middlePressed)
        updateView();
    vtkInteractorStyleTrackballCamera::OnMouseMove();

}

void MeasureStyle::OnLeftButtonDown()
{
    leftPressed = true;
    if(this->Interactor->GetControlKey()){

        this->cellPicker->AddPickList(mesh->getMeshSurfaceActor());
        this->cellPicker->PickFromListOn();
        //The click position of the mouse is taken
        int x, y;
        x = this->Interactor->GetEventPosition()[0];
        y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);
        //Some tolerance is set for the picking
        this->cellPicker->Pick(x, y, 0, meshRenderer);
        vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
        picker->Pick(x, y, 0, meshRenderer);

        vtkIdType cellID = this->cellPicker->GetCellId();

        if(measureType == MeasureType::BOUNDING || measureType == MeasureType::CALIBER)
        {
            vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
            coord->SetCoordinateSystemToDisplay();
            coord->SetValue(x, y, 0);
            double* worldCoord = coord->GetComputedWorldValue(meshRenderer);
            if(!measureStarted)
            {
                if(boundingBegin != nullptr)
                    delete boundingBegin;
                boundingBegin = new Point(worldCoord[0], worldCoord[1], worldCoord[2]);
                measureStarted = true;
            }
        } else if(cellID > 0 && cellID < this->mesh->T.numels()){

            double* pos = picker->GetPickPosition();
            Vertex* p = new Vertex(pos[0], pos[1], pos[2]);
            Vertex* v_i = mesh->getTriangle(cellID)->v1();
            Vertex* v = nullptr;
            double best_dist = DBL_MAX;
            for(unsigned int i = 0; i < 3; i++){
                double dist = ((*p) - (*v_i)).length();
                if(dist < best_dist){
                    v = v_i;
                    best_dist = dist;
                }
                v_i = mesh->getTriangle(cellID)->nextVertex(v_i);
            }

            for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++){
                if(mesh->getAnnotations()[i]->isPointInAnnotation(v) && dynamic_cast<DrawableAnnotation*>(mesh->getAnnotations()[i])->getSelected()){
                    if(measureType == MeasureType::HEIGHT)
                    {
                        manageHeightMovement(v);
                        last = v;
                    } else if(!measureStarted)
                    {
                        measurePath.clear();
                        measure = 0.0;
                        switch(measureType){
                            case MeasureType::RULER:
                            {
                                measurePath.push_back(v);
                                DrawableEuclideanMeasure* eucAtt = dynamic_cast<DrawableEuclideanMeasure*>(onCreationAttribute);
                                eucAtt->addMeasurePointID(mesh->getPointId(v));
                                break;
                            }
                            case MeasureType::TAPE:
                            {
                                DrawableGeodesicMeasure* geoAtt = dynamic_cast<DrawableGeodesicMeasure*>(onCreationAttribute);
                                geoAtt->addMeasurePointID(mesh->getPointId(v));
                                break;
                            }
                        }

                        measureStarted = true;
                        last = v;
                    } else
                    {
                        Vertex* start = last;
                        Vertex* end = v;
                        switch(measureType){
                            case MeasureType::RULER:
                                manageRulerMovement(start, end);
                                break;
                            case MeasureType::TAPE:
                                manageTapeMovement(start, end);
                                last = end;
                                break;
                        }
                    }

                    onCreationAttribute->update();
                    updateView();
                }
            }

        }
    }else
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();

}

void MeasureStyle::OnLeftButtonUp()
{
    leftPressed = false;
    if(measureStarted && measureType == MeasureType::BOUNDING)
        measureStarted = false;

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void MeasureStyle::OnRightButtonDown()
{
    if(this->Interactor->GetControlKey() && measureStarted && measureType != MeasureType::BOUNDING)
    {
        if(measurePath.size() > 0)
        {
            dynamic_cast<GeometricAttribute*>(onCreationAttribute)->removeMeasurePointID(mesh->getPointId(measurePath.back()));
            measurePath.pop_back();
            if(measurePath.size() > 0)
                last = measurePath.back();
            else
                measureStarted = false;
        } else
            measureStarted = false;
        onCreationAttribute->update();
        updateView();
    }

    vtkInteractorStyleTrackballCamera::OnRightButtonDown();

}

void MeasureStyle::OnMiddleButtonUp()
{
    middlePressed = false;
    vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
}

void MeasureStyle::OnMiddleButtonDown()
{
    middlePressed = true;
    if(measureStarted && this->Interactor->GetControlKey())
        reset();

    vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
}

void MeasureStyle::OnMouseWheelBackward()
{
    updateView();
    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
}

void MeasureStyle::OnMouseWheelForward()
{
    updateView();
    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
}

QVTKOpenGLNativeWidget *MeasureStyle::getQvtkwidget() const
{
    return qvtkwidget;
}

void MeasureStyle::setQvtkwidget(QVTKOpenGLNativeWidget *value)
{
    qvtkwidget = value;
}

vtkSmartPointer<vtkRenderer> MeasureStyle::getMeshRenderer() const
{
    return meshRenderer;
}

void MeasureStyle::setMeshRenderer(const vtkSmartPointer<vtkRenderer> &value)
{
    meshRenderer = value;
    meshRenderer->AddActor(measureAssembly);
}

MeasureStyle::MeasureType MeasureStyle::getMeasureType() const
{
    return measureType;
}

void MeasureStyle::setMeasureType(const MeasureType &value)
{
    measureType = value;
    reset();
}

double MeasureStyle::getMeasure() const
{
    return measure;
}

std::vector<IMATI_STL::Vertex *> MeasureStyle::getMeasurePath() const
{
    return measurePath;
}

DrawableAttribute *MeasureStyle::getOnCreationAttribute() const
{
    return onCreationAttribute;
}

bool MeasureStyle::getDrawAttributes() const
{
    return drawAttributes;
}

void MeasureStyle::setDrawAttributes(bool value)
{
    drawAttributes = value;
    updateView();
}
