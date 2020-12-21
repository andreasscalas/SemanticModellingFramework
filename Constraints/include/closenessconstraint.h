#ifndef CLOSENESSCONSTRAINT_H
#define CLOSENESSCONSTRAINT_H

#include <constraint.h>
namespace ShapeOp{
    class ClosenessConstraint : public Constraint
    {
    public:
        ClosenessConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions);
        virtual ~ClosenessConstraint() override {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        void setPosition(const Vector3 &position);
        Vector3 getPosition() const;
    private:
        Vector3 position;
    };
}

#endif // CLOSENESSCONSTRAINT_H
