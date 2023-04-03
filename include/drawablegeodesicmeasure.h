#ifndef DRAWABLEGEODESICMEASURE_H
#define DRAWABLEGEODESICMEASURE_H

#include <drawableattribute.h>
#include <geometricattribute.h>

class DrawableGeodesicMeasure : public GeometricAttribute, public DrawableAttribute
{
public:
    DrawableGeodesicMeasure();
	virtual ~DrawableGeodesicMeasure()  override {}
    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void print(std::ostream &) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &) override;

    const double MEASURE_DISTANCE = 0.1;

protected:
    std::vector<IMATI_STL::Point*> points;
    vtkSmartPointer<vtkActor> pointsActor;
    vtkSmartPointer<vtkActor> measureLineActor;
    vtkSmartPointer<vtkPoints> polylinePoints;
    vtkSmartPointer<vtkPoints> visualizedPoints;
    vtkSmartPointer<vtkCellArray> segments;

    double measureStringDistance;

};

#endif // DRAWABLEGEODESICMEASURE_H
