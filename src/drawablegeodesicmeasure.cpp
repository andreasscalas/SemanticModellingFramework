#include "drawablegeodesicmeasure.h"

#include <vtkPoints.h>

DrawableGeodesicMeasure::DrawableGeodesicMeasure(): GeometricAttribute(), DrawableAttribute()
{
    pointsActor = vtkSmartPointer<vtkActor>::New();
    measureLineActor =  vtkSmartPointer<vtkActor>::New();
}

void DrawableGeodesicMeasure::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    if(drawAttribute)
    {
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        DrawableAttribute::draw(canvas);
        canvas->AddPart(measureLineActor);
        canvas->AddPart(pointsActor);
        canvas->Modified();
        assembly->AddPart(canvas);
    }
    assembly->Modified();
}

void DrawableGeodesicMeasure::update()
{

    vtkSmartPointer<vtkPoints> polylinePoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> visualizedPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    measureLineActor =  vtkSmartPointer<vtkActor>::New();
    pointsActor = vtkSmartPointer<vtkActor>::New();
    points.clear();
    double measure = 0;
    if(measurePointsID.size() > 0)
    {
        for(unsigned int i = 0; i < measurePointsID.size(); i++)
        {
            IMATI_STL::Point* p = mesh->getPoint(measurePointsID[i]);
            points.push_back(p);
            vtkIdType last = polylinePoints->InsertNextPoint(p->x, p->y, p->z);
            if(i > 0)
            {
                measure += ((*p) - (*mesh->getPoint(measurePointsID[i - 1]))).length();
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetNumberOfIds(2);
                line->GetPointIds()->SetId(0, last - 1);
                line->GetPointIds()->SetId(1, last);
                cells->InsertNextCell(line);
            }
        }


        if(this->value == nullptr)
            this->value = new double(measure);
        else
            *static_cast<double*>(this->value) = measure;

        //Visualization of the polyline defining the measure
        vtkSmartPointer<vtkPolyData> measureLineData = vtkSmartPointer<vtkPolyData>::New();
        measureLineData->Initialize();
        measureLineData->SetPoints(polylinePoints);
        measureLineData->SetLines(cells);
        measureLineData->Modified();
        vtkSmartPointer<vtkPolyDataMapper> measureLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        measureLineMapper->SetInputData(measureLineData);
        measureLineMapper->Update();
        measureLineActor->SetMapper(measureLineMapper);
        measureLineActor->GetProperty()->SetColor(0.34, 0.651, 0.22);
        measureLineActor->GetProperty()->SetLineWidth(2.0f);

        //Visualization of the extreme points
            visualizedPoints->InsertNextPoint(points[0]->x, points[0]->y, points[0]->z);
        if(points.size() > 1)
            visualizedPoints->InsertNextPoint(points.back()->x, points.back()->y, points.back()->z);
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->Initialize();
        polydata->SetPoints(visualizedPoints);
        polydata->Modified();
        vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexFilter->SetInputData(polydata);
        vertexFilter->Update();
        vtkSmartPointer<vtkPolyDataMapper> pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        pointsMapper->SetInputConnection(vertexFilter->GetOutputPort());
        pointsActor->SetMapper(pointsMapper);
        pointsActor->GetProperty()->SetColor(0.34, 0.651, 0.22);
        pointsActor->GetProperty()->SetPointSize(10.0f);
    }
}

void DrawableGeodesicMeasure::print(std::ostream &writer)
{
    GeometricAttribute::print(writer);
    writer<< "Measure taken with a tape" << std::endl;

}

void DrawableGeodesicMeasure::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    GeometricAttribute::printJson(writer);
    writer.Key("tool");
    writer.String("tape");
}
