#ifndef ORIENTATIONCONSTRAINT_H
#define ORIENTATIONCONSTRAINT_H

#include <constraint.h>

namespace ShapeOp{
    class OrientationConstraint : public Constraint
    {
    public:
        OrientationConstraint(const std::vector<int> &idI,
                            double weight,
                            const std::shared_ptr<Matrix3X> &positions,
                            Vector3 normal = Vector3(0.0,0.0,1.0));
        virtual ~OrientationConstraint() {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        void setOrientation(const Vector3 &normal);

    private:
        Vector3 normal;
        mutable Matrix3X input;
    };
}
#endif // ORIENTATIONCONSTRAINT_H
