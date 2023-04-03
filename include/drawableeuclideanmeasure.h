#ifndef DRAWABLEEUCLIDEANMEASURE_H
#define DRAWABLEEUCLIDEANMEASURE_H

#include <drawableattribute.h>
#include <geometricattribute.h>
#include <vtkPoints.h>

class DrawableEuclideanMeasure : public GeometricAttribute, public DrawableAttribute
{
public:
    DrawableEuclideanMeasure();

    virtual ~DrawableEuclideanMeasure() override;

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void print(std::ostream &) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &) override;

protected:
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkPoints> points2D;
    vtkSmartPointer<vtkCellArray> measureSegmentCells;
    IMATI_STL::Point* p0;
    IMATI_STL::Point* p1;

};

#endif // DRAWABLEEUCLIDEANMEASURE_H
