#ifndef DRAWABLEANNOTATION_H
#define DRAWABLEANNOTATION_H

#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <annotation.h>

class DrawableAnnotation : virtual public Annotation {
	
public:

    DrawableAnnotation(){ meshPoints = vtkSmartPointer<vtkPoints>::New();
        annotationActor = vtkSmartPointer<vtkActor>::New();
        drawAttributes = true;
        selected = false;
        opacity = 0.7; }

    DrawableAnnotation(Annotation*){

    }

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) = 0;
    virtual void update() = 0;
    virtual void clear() = 0;

    vtkSmartPointer<vtkPoints> getMeshPoints() const { return meshPoints; }
    void setMeshPoints(const vtkSmartPointer<vtkPoints> &value) { meshPoints = value; }
    bool getSelected() const { return selected; }
    void setSelected(bool value) { selected = value; }

    vtkSmartPointer<vtkPropAssembly> getCanvas() const
    { return canvas; }

    bool getDrawAttributes() const
    { return drawAttributes; }

    void setDrawAttributes(bool value)
    { drawAttributes = value; }

    double getOpacity() const
    { return opacity; }

    void setOpacity(double value)
    { opacity = value; }
    
	virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) override = 0;
    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) override = 0;

    virtual void print(std::ostream& os) override{ Annotation::print(os); }
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer) override { Annotation::printJson(writer); }

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() override = 0;
    virtual bool isPointInAnnotation(IMATI_STL::Vertex* p) override = 0;


protected:
    vtkSmartPointer<vtkPoints> meshPoints;                     //Points data structure in VTK for the mesh
    vtkSmartPointer<vtkActor> annotationActor;
    vtkSmartPointer<vtkPropAssembly> canvas;

    double opacity;
    bool drawAttributes;                                       //True if the attributes has to be showed
    bool selected;
	
};

#endif // DRAWABLEANNOTATION_H
