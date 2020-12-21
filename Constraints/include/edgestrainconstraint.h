#ifndef EDGESTRAINCONSTRAINT_H
#define EDGESTRAINCONSTRAINT_H

#include <constraint.h>

namespace ShapeOp {

    class EdgeStrainConstraint : public Constraint {
    public:
        EdgeStrainConstraint(const std::vector<int> &idI,
                             double weight,
                             const std::shared_ptr<Matrix3X> positions,
                             double rangeMin = 1.0,
                             double rangeMax = 1.0);
        virtual ~EdgeStrainConstraint() {}
        /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        /** \brief Add the constraint to the linear system.*/
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        /** \brief Set a new edge length.*/
        void setEdgeLength(double length);
        /** \brief Set a new range minimum.*/
        void setRangeMin(double rMin) { rangeMin_ = rMin; }
        /** \brief Set a new range maximum.*/
        void setRangeMax(double rMax) { rangeMax_ = rMax; }
    private:
        double rest_;
        double rangeMin_;
        double rangeMax_;
    };

}
#endif // EDGESTRAINCONSTRAINT_H
