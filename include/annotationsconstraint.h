#ifndef ANNOTATIONSCONSTRAINT_H
#define ANNOTATIONSCONSTRAINT_H

#include <vector>

#include <annotation.h>
#include <constraintsolver.h>
#include <annotationsrelationship.h>

//ShapeOp constraint header inclusion
#include <Constraint.h>

class ConstraintSolver;
class AnnotationsConstraint : virtual public AnnotationsRelationship
{
public:
    AnnotationsConstraint();
    AnnotationsConstraint(AnnotationsRelationship*);

    virtual void constrain();
    virtual void checkConstraint();
    virtual void print(std::ostream &os);

    std::vector<Annotation *> getAnnotations() const;
    void setAnnotations(const std::vector<Annotation *> &value);
    double getError();

    ShapeOp::MatrixX3 *getPoints() const;
    void setPoints(ShapeOp::MatrixX3 *value);

    std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const;
    void setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value);

    ConstraintSolver *getSolver() const;
    void setSolver(ConstraintSolver *value);

protected:
    ShapeOp::MatrixX3* points;
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    ConstraintSolver* solver;

    void constrainSelf();
    void constrainSet();
    std::vector<int> getInvolvedVerticesIDs(Annotation* annotation);
};

#endif // ANNOTATIONSCONSTRAINT_H
