#ifndef ANNOTATIONMEASURESCONSTRAINT_H
#define ANNOTATIONMEASURESCONSTRAINT_H


#include <annotationmeasurerelationship.h>
#include <annotationsconstraint.h>

class AnnotationMeasuresConstraint: public AnnotationMeasuresRelationship, public AnnotationsConstraint
{

public:
    AnnotationMeasuresConstraint();

    virtual void constrain() override;
    virtual void print(std::ostream &os) override;
    virtual void checkConstraint() override;
};

#endif // ANNOTATIONMEASURESCONSTRAINT_H
