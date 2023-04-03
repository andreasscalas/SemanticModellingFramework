#include "drawableheightmeasure.h"

#include <vtkAxisActor2D.h>

DrawableHeightMeasure::DrawableHeightMeasure()
{
    points = vtkSmartPointer<vtkPoints>::New();
    direction = new IMATI_STL::Point();
    p1 = new IMATI_STL::Point();
}

DrawableHeightMeasure::~DrawableHeightMeasure()
{
    if(direction != nullptr)
        delete direction;
    if(p1 != nullptr)
        delete p1;
}

void DrawableHeightMeasure::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    if(drawAttribute)
    {

        if(measurePointsID.size() > 0)
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
            if(error)
                axisActor2D->GetProperty()->SetColor(1,0,0);
            else
                axisActor2D->GetProperty()->SetColor(0,0,0);
            axisActor2D->SetRulerMode(true);
            axisActor2D->GetProperty()->SetLineWidth(2.0);
            char string[512];
            sprintf(string, "%-#6.3g", ((*p0)-(*p1)).length());
            axisActor2D->SetTitle(string);
            canvas->AddPart(axisActor2D);
        }

        canvas->Modified();
        assembly->AddPart(canvas);
        assembly->Modified();
    }
    assembly->Modified();
}

void DrawableHeightMeasure::update()
{
    if(measurePointsID.size() < 1)
        return;
    IMATI_STL::Point * measurePin1 = mesh->getPoint(measurePointsID[0]);
    IMATI_STL::Point * measurePin2 = mesh->getLowestVertex(direction);
    if(measurePointsID.size() == 2)
        measurePointsID[1] = mesh->getPointId(static_cast<IMATI_STL::Vertex*>(measurePin2));
    else
        measurePointsID.push_back(mesh->getPointId(static_cast<IMATI_STL::Vertex*>(measurePin2)));
//    IMATI_STL::Point projectedPin1 = (*direction) * ((*measurePin1) * (*direction)) / ((*direction) * (*direction));
//    IMATI_STL::Point projectedPin2 = (*direction) * ((*measurePin1) * (*direction)) / ((*direction) * (*direction));
    double measure = (((*direction) / ((*direction) * (*direction))) * ((*measurePin1) * (*direction) - (*measurePin2) * (*direction))).length();
    p0 = measurePin1;
    p1->setValue((*p0) - (*direction) * measure);
    if(this->value == nullptr)
        this->value = new double(measure);
    else
        *static_cast<double*>(this->value) = measure;
    points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(p0->x, p0->y, p0->z);
    points->InsertNextPoint(p1->x, p1->y, p1->z);
}

void DrawableHeightMeasure::print(std::ostream &)
{

}

void DrawableHeightMeasure::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    GeometricAttribute::printJson(writer);
    writer.Key("tool");
    writer.String("height");
    writer.Key("direction");
    writer.StartArray();
    writer.Double(direction->x);
    writer.Double(direction->y);
    writer.Double(direction->z);
    writer.EndArray();
}

IMATI_STL::Point *DrawableHeightMeasure::getOrigin() const
{
    return p0;
}

void DrawableHeightMeasure::setOrigin(IMATI_STL::Point *value)
{
    p0 = value;
}

IMATI_STL::Point *DrawableHeightMeasure::getBase() const
{
    return p1;
}

IMATI_STL::Point *DrawableHeightMeasure::getDirection() const
{
    return direction;
}

void DrawableHeightMeasure::setDirection(IMATI_STL::Point *value)
{
    direction->setValue(value->x, value->y, value->z);
}

void DrawableHeightMeasure::setDirection(double x, double y, double z)
{
    direction->setValue(x, y, z);
}

