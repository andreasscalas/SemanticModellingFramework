#ifndef SEMANTICCONSTRAINT_H
#define SEMANTICCONSTRAINT_H

#include <vector>
#include <Constraint.h>
#include <constraintsolver.h>

class ConstraintSolver;
class SemanticConstraint
{
public:
    SemanticConstraint() {}
    virtual ~SemanticConstraint() {}
    virtual void constrain() = 0;
    virtual double getError() = 0;
    ShapeOp::MatrixX3 *getPoints() const { return points; }
    void setPoints(ShapeOp::MatrixX3 *value){ points = value; }
    double getWeight() const { return weight; }
    void setWeight(double value){ weight = value; }
    double getMinValue() const{ return minValue; }
    void setMinValue(double value){ minValue = value; }
    double getMaxValue() const{ return maxValue; }
    void setMaxValue(double value){ maxValue = value; }
    std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const{ return constraints; }
    ConstraintSolver *getSolver() const{ return solver;}
    void setSolver(ConstraintSolver *value){ solver = value; }

protected:
    ShapeOp::MatrixX3* points;
    double weight;
    double minValue, maxValue;
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    ConstraintSolver* solver;
};

#endif // SEMANTICCONSTRAINT_H
