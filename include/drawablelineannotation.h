#ifndef DRAWABLELINEANNOTATION_H
#define DRAWABLELINEANNOTATION_H
#include <lineannotation.h>
#include <drawableannotation.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>

class DrawableLineAnnotation : public LineAnnotation, public DrawableAnnotation
{
public:
    DrawableLineAnnotation();
    DrawableLineAnnotation(LineAnnotation*);
    ~DrawableLineAnnotation() override {}

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void clear() override;

    float getLineWidth() const;
    void setLineWidth(float value);

    virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) override;
    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    virtual void print(std::ostream& os) override { LineAnnotation::print(os); }
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer) override { LineAnnotation::printJson(writer); }

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() override;
    virtual bool isPointInAnnotation(IMATI_STL::Vertex* p) override;


protected:

    void init();

    vtkSmartPointer<vtkCellArray> annotatedLines;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    float lineWidth = 3.0;

};

#endif // DRAWABLELINEANNOTATION_H
