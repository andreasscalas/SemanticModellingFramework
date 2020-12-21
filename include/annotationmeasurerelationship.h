#ifndef ANNOTATIONMEASURESRELATIONSHIP_H
#define ANNOTATIONMEASURESRELATIONSHIP_H

#include <annotationsrelationship.h>
#include <geometricattribute.h>

class AnnotationMeasuresRelationship : virtual public AnnotationsRelationship
{
public:
    AnnotationMeasuresRelationship();

    virtual ~AnnotationMeasuresRelationship();
    GeometricAttribute *getAttribute1() const;
    void setAttribute1(GeometricAttribute *value);

    GeometricAttribute *getAttribute2() const;
    void setAttribute2(GeometricAttribute *value);

    virtual void print(std::ostream &os) override;
    virtual void printJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer) override;

protected:
    GeometricAttribute * attribute1, * attribute2;
};

#endif // ANNOTATIONMEASURESRELATIONSHIP_H
