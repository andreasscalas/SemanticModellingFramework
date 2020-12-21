#include "drawableeuclideanmeasure.h"

#include <vtkProperty2D.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>


DrawableEuclideanMeasure::DrawableEuclideanMeasure() : GeometricAttribute(), DrawableAttribute()
{
    points = vtkSmartPointer<vtkPoints>::New();
}

DrawableEuclideanMeasure::~DrawableEuclideanMeasure()
{
}

void DrawableEuclideanMeasure::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    if(drawAttribute)
    {
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        DrawableAttribute::draw(canvas);

        //2D part for always showing measure segment
        if(measurePointsID.size() > 1)
        {
            points2D = vtkSmartPointer<vtkPoints>::New();
            measureSegmentCells = vtkSmartPointer<vtkCellArray>::New();
            points2D->SetNumberOfPoints(2);
            vtkCoordinate *coordinate = vtkCoordinate::New();
            coordinate->SetCoordinateSystemToWorld();
            coordinate->SetValue(p0->x, p0->y, p0->z);
            double* pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
            vtkIdType firstID = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
            coordinate->SetValue(p1->x, p1->y, p1->z);
            pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
            vtkIdType secondID = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetNumberOfIds(2);
            line->GetPointIds()->SetId(0, firstID);
            line->GetPointIds()->SetId(1, secondID);
            measureSegmentCells->InsertNextCell(line);
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

        }

        if(measurePointsID.size() > 0)
        {
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
        }
        canvas->Modified();
        assembly->AddPart(canvas);
    }
    assembly->Modified();

}

void DrawableEuclideanMeasure::update()
{

    double measure = 0;
    points = vtkSmartPointer<vtkPoints>::New();
    p0 = nullptr;
    p1 = nullptr;
    if(measurePointsID.size() > 0)
    {
        p0 = mesh->getPoint(measurePointsID[0]);
        points->InsertNextPoint(p0->x, p0->y, p0->z);
        if(measurePointsID.size() > 1)
        {
            p1 = mesh->getPoint(measurePointsID[1]);
            points->InsertNextPoint(p1->x, p1->y, p1->z);
            measure = ((*p1) - (*p0)).length();
        }
    }

    if(this->value == nullptr)
        this->value = new double(measure);
    else
        *static_cast<double*>(this->value) = measure;
}

void DrawableEuclideanMeasure::print(std::ostream &writer)
{
    GeometricAttribute::print(writer);
    writer<< "Measure taken with a ruler" << std::endl;
}

void DrawableEuclideanMeasure::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer )
{
    GeometricAttribute::printJson(writer);
    writer.Key("tool");
    writer.String("ruler");
}
