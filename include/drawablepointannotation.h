#ifndef DRAWABLEPOINTANNOTATION_H
#define DRAWABLEPOINTANNOTATION_H

#include <pointannotation.h>
#include <drawableannotation.h>


class DrawablePointAnnotation : public PointAnnotation, public DrawableAnnotation
{
public:
    DrawablePointAnnotation();
    DrawablePointAnnotation(PointAnnotation*);
    ~DrawablePointAnnotation();

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly);
    virtual void update();
    virtual void clear();
    float getPointSize() const;
    void setPointSize(float value);

protected:
    void init();

    vtkSmartPointer<vtkPoints> annotatedPoints;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    float pointSize = 10.0;
};

#endif // DRAWABLEPOINTANNOTATION_H
