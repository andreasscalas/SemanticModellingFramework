#include "drawableboundingmeasure.h"

#include <annotationutilities.h>

#include <vtkProperty2D.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkLine.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkAxisActor2D.h>

DrawableBoundingMeasure::DrawableBoundingMeasure() : GeometricAttribute(), DrawableAttribute()
{
    points = vtkSmartPointer<vtkPoints>::New();
    direction = nullptr;
    drawPlanes = false;
    planeSize = 0.0;
}

DrawableBoundingMeasure::~DrawableBoundingMeasure()
{
    if(direction != nullptr)
        delete direction;
}

void DrawableBoundingMeasure::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    if(drawAttribute)
    {

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
            axisActor2D->GetPoint1Coordinate()->SetValue(extreme0.x, extreme0.y, extreme0.z);
            axisActor2D->GetPoint2Coordinate()->SetValue(extreme1.x, extreme1.y, extreme1.z);
            axisActor2D->SetRulerMode(true);
            axisActor2D->GetProperty()->SetLineWidth(2.0);
            if(error)
                axisActor2D->GetProperty()->SetColor(1,0,0);
            else
                axisActor2D->GetProperty()->SetColor(0,0,0);
            char string[512];
            sprintf(string, "%-#6.3g", ((extreme0) - (extreme1)).length());
            axisActor2D->SetTitle(string);
            canvas->AddPart(axisActor2D);


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
        assembly->Modified();
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
    *p.first *= measure * 5;
    *p.second *= measure * 5;
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
