#ifndef DRAWABLEBOUNDINGMEASURE_H
#define DRAWABLEBOUNDINGMEASURE_H

#include <drawableattribute.h>
#include <geometricattribute.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>

class DrawableBoundingMeasure:  public GeometricAttribute,  public DrawableAttribute
{
public:
    DrawableBoundingMeasure();

    virtual ~DrawableBoundingMeasure() override;

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void print(std::ostream &) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &) override;

    IMATI_STL::Point *getOrigin() const;
    void setOrigin(IMATI_STL::Point *value);

    IMATI_STL::Point *getBoundingBegin() const;
    void setBoundingBegin(IMATI_STL::Point *value);

    IMATI_STL::Point *getBoundingEnd() const;
    void setBoundingEnd(IMATI_STL::Point *value);

    IMATI_STL::Point *getDirection() const;
    void setDirection(IMATI_STL::Point *value);
    void setDirection(IMATI_STL::Point value);

    bool getDrawPlanes() const;
    void setDrawPlanes(bool value);


protected:
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkPoints> points2D;
    vtkSmartPointer<vtkCellArray> measureSegmentCells;
    vtkSmartPointer<vtkPlaneSource> planeSource1;
    vtkSmartPointer<vtkPlaneSource> planeSource2;
    IMATI_STL::Point * origin;
    IMATI_STL::Point * direction;
    IMATI_STL::Point extreme0;
    IMATI_STL::Point extreme1;
    bool drawPlanes;
    double planeSize;

};

#endif // DRAWABLEBOUNDINGMEASURE_H
