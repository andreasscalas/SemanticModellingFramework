#ifndef DRAWABLEANNOTATION_H
#define DRAWABLEANNOTATION_H

#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <annotation.h>
#include <drawableattribute.h>

class DrawableAnnotation : virtual public Annotation {
public:

    DrawableAnnotation(){
        meshPoints = vtkSmartPointer<vtkPoints>::New();
        annotationActor = vtkSmartPointer<vtkActor>::New();
        drawAttributes = true;
        selected = false;
    }

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) = 0;
    virtual void update() = 0;
    virtual void clear() = 0;

    vtkSmartPointer<vtkPoints> getMeshPoints() const { return meshPoints; }
    void setMeshPoints(const vtkSmartPointer<vtkPoints> &value) { meshPoints = value; }
    bool getSelected() const { return selected; }
    void setSelected(bool value) { selected = value; }

    vtkSmartPointer<vtkPropAssembly> getCanvas() const
    {
        return canvas;
    }

    bool getDrawAttributes() const
    {
        return drawAttributes;
    }

    void setDrawAttributes(bool value)
    {
        drawAttributes = value;
    }

protected:
    vtkSmartPointer<vtkPoints> meshPoints;                     //Points data structure in VTK for the mesh
    vtkSmartPointer<vtkActor> annotationActor;
    vtkSmartPointer<vtkPropAssembly> canvas;

    bool drawAttributes;                                       //True if the attributes has to be showed
    bool selected;
};

#endif // DRAWABLEANNOTATION_H


