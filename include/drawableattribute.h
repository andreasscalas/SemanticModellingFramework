#ifndef DRAWABLEATTRIBUTE_H
#define DRAWABLEATTRIBUTE_H

#include <attribute.h>
#include <drawablemesh.h>
#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVectorText.h>
#include <string>
#include <vtkMapper2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>

class DrawableMesh;

class DrawableAttribute : public virtual Attribute
{

public:

    DrawableAttribute() {
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        drawAttribute = true;
        drawValue = true;
        error = false;
    }

    virtual ~DrawableAttribute() override { mesh = nullptr; }

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) = 0;

    virtual void update() = 0;

    DrawableMesh *getMesh() const{
        return mesh;
    }

    void setMesh(DrawableMesh *mesh)
    {
        this->mesh = mesh;
    }

    vtkSmartPointer<vtkRenderer> getRenderer() const
    {
        return renderer;
    }

    void setRenderer(const vtkSmartPointer<vtkRenderer> &ren)
    {
        renderer = ren;
    }

    bool getDrawAttribute() const
    {
        return drawAttribute;
    }

    void setDrawAttribute(bool value)
    {
        drawAttribute = value;
    }

    bool getDrawValue() const
    {
        return drawValue;
    }

    void setDrawValue(bool value)
    {
        drawValue = value;
    }

    vtkSmartPointer<vtkPropAssembly> getCanvas() const{
        return canvas;
    }

    void setCanvas(const vtkSmartPointer<vtkPropAssembly> &value){
        canvas = value;
    }

    bool getError() const{
        return error;
    }
    void setError(bool value){
        error = value;
    }

protected:
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    bool drawAttribute;
    bool drawValue;
    bool error;
    DrawableMesh* mesh;
    std::pair<unsigned int, unsigned int> textPosition;
};


#endif // DRAWABLEATTRIBUTE_H

