#include "drawableeuclideanmeasure.h"

#include <vtkProperty2D.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkLine.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkAxisActor2D.h>

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
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    if(drawAttribute)
    {

        //2D part for always showing measure segment
        if(measurePointsID.size() > 1)
        {
            vtkSmartPointer<vtkAxisActor2D> axisActor2D = vtkSmartPointer<vtkAxisActor2D>::New();
            axisActor2D->GetPoint1Coordinate()->SetCoordinateSystemToWorld();
            axisActor2D->GetPoint2Coordinate()->SetCoordinateSystemToWorld();
            axisActor2D->SetNumberOfLabels(5);
            axisActor2D->LabelVisibilityOff();
            axisActor2D->AdjustLabelsOff();
            axisActor2D->SetTitle("Distance");
            axisActor2D->GetTitleTextProperty()->SetBold(1);
            axisActor2D->GetTitleTextProperty()->SetItalic(1);
            axisActor2D->GetTitleTextProperty()->SetShadow(1);
            axisActor2D->GetTitleTextProperty()->SetFontFamilyToArial();
            axisActor2D->GetTitleTextProperty()->SetFontSize(16);
            if(error)
                axisActor2D->GetTitleTextProperty()->SetColor(1,0,0);
            else
                axisActor2D->GetTitleTextProperty()->SetColor(0,0,0);
            axisActor2D->SetFontFactor(1.0);
            axisActor2D->SetLabelFactor(1.0);
            axisActor2D->GetPoint1Coordinate()->SetValue(p0->x, p0->y, p0->z);
            axisActor2D->GetPoint2Coordinate()->SetValue(p1->x, p1->y, p1->z);
            axisActor2D->SetRulerMode(true);
            axisActor2D->GetProperty()->SetLineWidth(2.0);
            if(error)
                axisActor2D->GetProperty()->SetColor(1,0,0);
            else
                axisActor2D->GetProperty()->SetColor(0,0,0);

            char string[512];
            sprintf(string, "%-#6.3g", ((*p0)-(*p1)).length());
            axisActor2D->SetTitle(string);
            canvas->AddPart(axisActor2D);
//            points2D = vtkSmartPointer<vtkPoints>::New();
//            measureSegmentCells = vtkSmartPointer<vtkCellArray>::New();
//            points2D->SetNumberOfPoints(2);
//            vtkCoordinate *coordinate = vtkCoordinate::New();
//            coordinate->SetCoordinateSystemToWorld();
//            coordinate->SetValue(p0->x, p0->y, p0->z);
//            double* pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
//            vtkIdType firstID = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
//            coordinate->SetValue(p1->x, p1->y, p1->z);
//            pos2D = coordinate->GetComputedDoubleDisplayValue(renderer);
//            vtkIdType secondID = points2D->InsertNextPoint(pos2D[0], pos2D[1], 0);
//            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
//            line->GetPointIds()->SetNumberOfIds(2);
//            line->GetPointIds()->SetId(0, firstID);
//            line->GetPointIds()->SetId(1, secondID);
//            measureSegmentCells->InsertNextCell(line);
//            vtkSmartPointer<vtkPolyData> measureLineData = vtkSmartPointer<vtkPolyData>::New();
//            measureLineData->Initialize();
//            measureLineData->SetPoints(points2D);
//            measureLineData->SetLines(measureSegmentCells);
//            measureLineData->Modified();
//            vtkSmartPointer<vtkPolyDataMapper2D> measureLineMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
//            measureLineMapper->SetInputData(measureLineData);
//            measureLineMapper->Update();
//            vtkSmartPointer<vtkActor2D> measureLineActor = vtkSmartPointer<vtkActor2D>::New();
//            measureLineActor->SetMapper(measureLineMapper);
//            measureLineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
//            measureLineActor->GetProperty()->SetLineWidth(2.0f);
//            canvas->AddPart(measureLineActor);
//            //3D points that should be shown only when visible
//            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//            polydata->Initialize();
//            polydata->SetPoints(points);
//            polydata->Modified();
//            vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
//            vertexFilter->SetInputData(polydata);
//            vertexFilter->Update();
//            vtkSmartPointer<vtkPolyDataMapper> pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//            vtkSmartPointer<vtkActor> pointsActor = vtkSmartPointer<vtkActor>::New();
//            pointsMapper->SetInputConnection(vertexFilter->GetOutputPort());
//            pointsActor->SetMapper(pointsMapper);
//            pointsActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
//            pointsActor->GetProperty()->SetPointSize(10.0f);
//            canvas->AddPart(pointsActor);
        }

        canvas->Modified();
        assembly->AddPart(canvas);
        assembly->Modified();
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
