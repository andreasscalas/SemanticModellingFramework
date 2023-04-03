#ifndef DRAWABLEHEIGHTMEASURE_H
#define DRAWABLEHEIGHTMEASURE_H


#include <drawableattribute.h>
#include <geometricattribute.h>
#include <vtkPoints.h>

class DrawableHeightMeasure : public GeometricAttribute, public DrawableAttribute
{
public:
    DrawableHeightMeasure();

    virtual ~DrawableHeightMeasure() override;

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void print(std::ostream &) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &) override;

    IMATI_STL::Point *getOrigin() const;
    void setOrigin(IMATI_STL::Point *value);


    IMATI_STL::Point *getBase() const;

    IMATI_STL::Point *getDirection() const;
    void setDirection(IMATI_STL::Point *value);
    void setDirection(double, double, double);

protected:
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkCellArray> measureSegmentCells;
    IMATI_STL::Point* p0;
    IMATI_STL::Point* p1;
    IMATI_STL::Point* direction;
};

#endif // DRAWABLEHEIGHTMEASURE_H
