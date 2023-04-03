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
	~DrawableAreaAnnotation() override {}

    virtual void draw(vtkSmartPointer<vtkPropAssembly> canvas) override;
    virtual void update() override;
    virtual void clear() override;
	
    virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) override;
    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    virtual void print(std::ostream& os) override{ SurfaceAnnotation::print(os); }
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)  override{ SurfaceAnnotation::printJson(writer); }

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() override;
    virtual bool isPointInAnnotation(IMATI_STL::Vertex* p) override;

protected:

    void init();
    vtkSmartPointer<vtkCellArray> annotatedTriangles;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    vtkSmartPointer<vtkActor> outlinesActor;
};

#endif // DRAWABLEAREAANNOTATION_H
