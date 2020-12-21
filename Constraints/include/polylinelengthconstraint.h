#ifndef POLYLINELENGTHCONSTRAINT_H
#define POLYLINELENGTHCONSTRAINT_H

#include <constraint.h>
namespace  ShapeOp{

    class PolylineLengthConstraint : public Constraint
    {
    public:
        PolylineLengthConstraint(const std::vector<int> &idI,
                            double weight,
                            const std::shared_ptr<Matrix3X> positions,
                            const double minRange,
                            const double maxRange);
        virtual ~PolylineLengthConstraint() {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        double getRangeMin() const;
        void setRangeMin(const double &value);

        double getRangeMax() const;
        void setRangeMax(const double &value);

    private:
        mutable Matrix3X input;
        double rangeMin;
        double rangeMax;
    };

}
#endif // POLYLINELENGTHCONSTRAINT_H
