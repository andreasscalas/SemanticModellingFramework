#ifndef SIMILARITYCONSTRAINT_H
#define SIMILARITYCONSTRAINT_H

#include <constraint.h>

namespace ShapeOp{
    class SimilarityConstraint : public Constraint {
    public:
        SimilarityConstraint(const std::vector<int> &idI,
                           double weight,
                           const std::shared_ptr<Matrix3X> positions,
                           bool scaling = true,
                           bool rotate = true,
                           bool flip = true);
        virtual ~SimilarityConstraint() override {}
        virtual void project(std::shared_ptr<MatrixX3> projections) const override final;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        void setShapes( const std::vector<Matrix3X> &shapes );
    private:
        void testCandidates(double &min_error) const;
        mutable Matrix3X input;
        mutable Matrix3X candidate;
        mutable Matrix3X output;
        std::vector<Matrix3X>  shapes_;
        bool scaling_;
        bool rotate_;
        bool flip_;
        mutable Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permutation_;
        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> rotateMatrix_;
        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> flipMatrix_;
    };
}

#endif // SIMILARITYCONSTRAINT_H
