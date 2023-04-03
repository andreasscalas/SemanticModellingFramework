#ifndef DRAWABLEPOINTANNOTATION_H
#define DRAWABLEPOINTANNOTATION_H

#include <pointannotation.h>
#include <drawableannotation.h>
#include <vtkUnsignedCharArray.h>


class DrawablePointAnnotation : public PointAnnotation, public DrawableAnnotation
{
public:
    DrawablePointAnnotation();
    DrawablePointAnnotation(PointAnnotation*);
    virtual ~DrawablePointAnnotation() override;

    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly) override;
    virtual void update() override;
    virtual void clear() override;
    float getPointSize() const;
    void setPointSize(float value);
    virtual Annotation * transfer(ExtendedTrimesh *otherMesh, short metric) override;
    virtual Annotation * parallelTransfer(ExtendedTrimesh *otherMesh, short metric) override;

    virtual void print(std::ostream& os) override { PointAnnotation::print(os); }
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer) override { PointAnnotation::printJson(writer); }

    virtual std::vector<IMATI_STL::Vertex *> getInvolvedVertices() override;
    virtual bool isPointInAnnotation(IMATI_STL::Vertex *p) override;

protected:
    void init();

    vtkSmartPointer<vtkPoints> annotatedPoints;          //Data structure in VTK for the annotated triangles
    vtkSmartPointer<vtkUnsignedCharArray> annotationColors;   //Array of colors of the annotated triangles
    float pointSize = 10.0;
};

#endif // DRAWABLEPOINTANNOTATION_H
