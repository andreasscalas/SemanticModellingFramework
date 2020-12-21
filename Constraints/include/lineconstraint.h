#ifndef LINECONSTRAINT_H
#define LINECONSTRAINT_H


#include <constraint.h>

namespace ShapeOp {
    class LineConstraint : public Constraint
    {
    public:
        LineConstraint(const std::vector<int> &idI,
                       double weight,
                       const std::shared_ptr<Matrix3X> positions);
        virtual ~LineConstraint() {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
    private:
        mutable Matrix3X input;
    };
}

#endif // LINECONSTRAINT_H
