#include "drawableboundingmeasure.h"

#include <vtkProperty2D.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <annotationutilities.h>

DrawableBoundingMeasure::DrawableBoundingMeasure() : GeometricAttribute(), DrawableAttribute()
{
    points = vtkSmartPointer<vtkPoints>::New();
    direction = nullptr;
    drawPlanes = false;
}

DrawableBoundingMeasure::~DrawableBoundingMeasure()
{
    if(direction != nullptr)
        delete direction;
}

void DrawableBoundingMeasure::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    if(drawAttribute)
    {
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        DrawableAttribute::draw(canvas);

        if(measurePointsID.size() > 1)
        {
            points2D = vtkSmartPointer<vtkPoints>::New();
            measureSegmentCells = vtkSmartPointer<vtkCellArray>::New();
            points2D->SetNumberOfPoints(2);
            vtkCoordinate *coordinate = vtkCoordinate::New();
            coordinate->SetCoordinateSystemToWorld();
            coordinate->SetValue(extreme0.x, extreme0.y, extreme0.z);
            double* pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
            vtkIdType id1 = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
            coordinate->SetValue(extreme1.x, extreme1.y, extreme1.z);
            pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
            vtkIdType id2 = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetNumberOfIds(2);
            line->GetPointIds()->SetId(0, id1);
            line->GetPointIds()->SetId(1, id2);
            measureSegmentCells->InsertNextCell(line);
            //2D part for always showing measure segment
            vtkSmartPointer<vtkPolyData> measureLineData = vtkSmartPointer<vtkPolyData>::New();
            measureLineData->Initialize();
            measureLineData->SetPoints(points2D);
            measureLineData->SetLines(measureSegmentCells);
            measureLineData->Modified();
            vtkSmartPointer<vtkPolyDataMapper2D> measureLineMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
            measureLineMapper->SetInputData(measureLineData);
            measureLineMapper->Update();
            vtkSmartPointer<vtkActor2D> measureLineActor = vtkSmartPointer<vtkActor2D>::New();
            measureLineActor->SetMapper(measureLineMapper);
            measureLineActor->GetProperty()->SetColor(0.34, 0.651, 0.22);
            measureLineActor->GetProperty()->SetLineWidth(2.0f);
            canvas->AddPart(measureLineActor);


            //3D points that should be shown only when visible
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            polydata->Initialize();
            polydata->SetPoints(points);
            polydata->Modified();
            vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
            vertexFilter->SetInputData(polydata);
            vertexFilter->Update();
            vtkSmartPointer<vtkPolyDataMapper> pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkActor> pointsActor = vtkSmartPointer<vtkActor>::New();
            pointsMapper->SetInputConnection(vertexFilter->GetOutputPort());
            pointsActor->SetMapper(pointsMapper);
            pointsActor->GetProperty()->SetColor(0.34, 0.651, 0.22);
            pointsActor->GetProperty()->SetPointSize(10.0f);
            canvas->AddPart(pointsActor);

            if(drawPlanes)
            {
                vtkSmartPointer<vtkPolyDataMapper> planeMapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
                planeMapper1->SetInputConnection(planeSource1->GetOutputPort());
                vtkSmartPointer<vtkActor> planeActor1 = vtkSmartPointer<vtkActor>::New();
                planeActor1->SetMapper(planeMapper1);
                canvas->AddPart(planeActor1);
                vtkSmartPointer<vtkPolyDataMapper> planeMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
                planeMapper2->SetInputConnection(planeSource2->GetOutputPort());
                vtkSmartPointer<vtkActor> planeActor2 = vtkSmartPointer<vtkActor>::New();
                planeActor2->SetMapper(planeMapper2);
                canvas->AddPart(planeActor2);
            }
        }

        canvas->Modified();
        assembly->AddPart(canvas);
    }
    assembly->Modified();
}

void DrawableBoundingMeasure::update()
{
    if(origin == nullptr)
        return;

    if(measurePointsID.size() < 2)
        return;
    IMATI_STL::Point * p0 = mesh->getPoint(measurePointsID[0]);
    IMATI_STL::Point * p1 = mesh->getPoint(measurePointsID[1]);
    extreme0 = p0;
    extreme0 -= origin;
    extreme0 = (*direction) * (extreme0 * (*direction));
    extreme0 += origin;
    extreme1 = p1;
    extreme1 -= origin;
    extreme1 = (*direction) * (extreme1 * (*direction));
    extreme1 += origin;

    double measure = (extreme1 - extreme0).length();
    if(this->value == nullptr)
        this->value = new double(measure);
    else
        *static_cast<double*>(this->value) = measure;
    points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(extreme0.x, extreme0.y, extreme0.z);
    points->InsertNextPoint(extreme1.x, extreme1.y, extreme1.z);
    planeSource1 = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource2 = vtkSmartPointer<vtkPlaneSource>::New();
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> p = Utilities::compute2OrthogonalVersors(*direction);
    *p.first *= 10;
    *p.second *= 10;
    IMATI_STL::Point o1 = extreme0 - (*p.first) - (*p.second);
    IMATI_STL::Point p1_1 = o1 + (*p.first) * 2;
    IMATI_STL::Point p2_1 = o1 + (*p.second) * 2;
    planeSource1->SetOrigin(o1.x, o1.y, o1.z);
    planeSource1->SetPoint1(p1_1.x, p1_1.y, p1_1.z);
    planeSource1->SetPoint2(p2_1.x, p2_1.y, p2_1.z);
    planeSource1->Update();
    IMATI_STL::Point o2 = extreme1 - (*p.first) - (*p.second);
    IMATI_STL::Point p1_2 = o2 + (*p.first) * 2;
    IMATI_STL::Point p2_2 = o2 + (*p.second) * 2;
    planeSource2->SetOrigin(o2.x, o2.y, o2.z);
    planeSource2->SetPoint1(p1_2.x, p1_2.y, p1_2.z);
    planeSource2->SetPoint2(p2_2.x, p2_2.y, p2_2.z);
    planeSource2->Update();

}

void DrawableBoundingMeasure::print(std::ostream &writer)
{
    GeometricAttribute::print(writer);
    writer<< "Measure taken with a bounding tool. It is defined in the direction: (" <<
             direction->x << "," << direction->y << "," << direction->z << ")"<< std::endl;
}

void DrawableBoundingMeasure::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    GeometricAttribute::printJson(writer);
    writer.Key("tool");
    writer.String("bounding");
    writer.Key("direction");
    writer.StartArray();
    writer.Double(direction->x);
    writer.Double(direction->y);
    writer.Double(direction->z);
    writer.EndArray();
}

IMATI_STL::Point *DrawableBoundingMeasure::getOrigin() const
{
    return origin;
}

void DrawableBoundingMeasure::setOrigin(IMATI_STL::Point *origin)
{
    this->origin = origin;
}


IMATI_STL::Point *DrawableBoundingMeasure::getDirection() const
{
    return direction;
}

void DrawableBoundingMeasure::setDirection(IMATI_STL::Point *direction)
{
    this->direction = direction;
}

void DrawableBoundingMeasure::setDirection(T_MESH::Point direction)
{
    this->direction = new IMATI_STL::Point(direction);
}

bool DrawableBoundingMeasure::getDrawPlanes() const
{
    return drawPlanes;
}

void DrawableBoundingMeasure::setDrawPlanes(bool value)
{
    drawPlanes = value;
}
