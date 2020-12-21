#ifndef DRAWABLEATTRIBUTE_H
#define DRAWABLEATTRIBUTE_H

#include <attribute.h>
#include <drawablemesh.h>
#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <string>

class DrawableMesh;

class DrawableAttribute : public virtual Attribute
{

public:

    DrawableAttribute() {
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        drawAttribute = true;
    }

    virtual ~DrawableAttribute() {mesh = nullptr;}

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly)
    {
        assembly->RemovePart(canvas);
        canvas = vtkSmartPointer<vtkPropAssembly>::New();
        vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
        textActor->SetInput( std::to_string(*static_cast<double*>(value)).c_str());
        //textActor->SetPosition2 ( 10, 40 );
        textActor->GetTextProperty()->SetFontSize ( 24 );
        textActor->GetTextProperty()->SetColor ( 1.0, 0.0, 0.0 );
        canvas->AddPart(textActor);
        assembly->AddPart(canvas);
    }

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

protected:
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    bool drawAttribute;
    DrawableMesh* mesh;
};
#endif // DRAWABLEATTRIBUTE_H
