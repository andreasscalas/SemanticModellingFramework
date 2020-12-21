#ifndef PLANECONSTRAINT_H
#define PLANECONSTRAINT_H


#include <constraint.h>
namespace ShapeOp{
    class PlaneConstraint : public Constraint
    {
    public:
        PlaneConstraint(const std::vector<int> &idI,
                        double weight,
                        const std::shared_ptr<Matrix3X> positions);
        virtual ~PlaneConstraint() {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
    private:
        mutable Matrix3X input;
    };
}

#endif // PLANECONSTRAINT_H
