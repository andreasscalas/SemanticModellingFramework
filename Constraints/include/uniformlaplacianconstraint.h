#ifndef UNIFORMLAPLACIANCONSTRAINT_H
#define UNIFORMLAPLACIANCONSTRAINT_H
#include <constraint.h>
namespace ShapeOp {
    class UniformLaplacianConstraint : public Constraint
    {
    public:
        UniformLaplacianConstraint(const std::vector<int> &idI,
                                   double weight,
                                   const std::shared_ptr<Matrix3X> positions,
                                   bool displacement_lap);
        virtual ~UniformLaplacianConstraint() {}
        /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        /** \brief Add the constraint to the linear system.*/
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        Vector3 getDisplacement() const;
        void setDisplacement(const Vector3 &getDisplacement);

      private:
        Vector3 displacement;
    };

}
#endif // UNIFORMLAPLACIANCONSTRAINT_H
