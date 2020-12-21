#ifndef DRAWABLEAREAANNOTATION_H
#define DRAWABLEAREAANNOTATION_H

#include <surfaceannotation.h>
#include <drawableannotation.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>

class DrawableAreaAnnotation : public SurfaceAnnotation, public DrawableAnnotation
{
public:
    DrawableAreaAnnotation();
    DrawableAreaAnnotation(SurfaceAnnotation* annotation);

    virtual void draw(vtkSmartPointer<vtkPropAssembly> canvas);
    virtual void update();
    virtual void clear();

protected:

    void init();
    vtkSmartPointer<vtkCellArray> annotatedTriangles;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    vtkSmartPointer<vtkActor> outlinesActor;
};

#endif // DRAWABLEAREAANNOTATION_H
