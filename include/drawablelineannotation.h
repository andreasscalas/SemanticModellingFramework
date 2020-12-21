#ifndef DRAWABLELINEANNOTATION_H
#define DRAWABLELINEANNOTATION_H
#include <lineannotation.h>
#include <drawableannotation.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>

class DrawableLineAnnotation : public LineAnnotation, public DrawableAnnotation
{
public:
    DrawableLineAnnotation();
    DrawableLineAnnotation(LineAnnotation*);

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly);
    virtual void update();
    virtual void clear();

    float getLineWidth() const;
    void setLineWidth(float value);

protected:

    void init();

    vtkSmartPointer<vtkCellArray> annotatedLines;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    float lineWidth = 3.0;

};

#endif // DRAWABLELINEANNOTATION_H
