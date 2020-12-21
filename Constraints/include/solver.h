#ifndef SOLVER_H
#define SOLVER_H

#include <constraint.h>
#include <types.h>
#include <memory>

namespace ShapeOp{

    class Solver
    {
    public:
        int addConstraint(const std::shared_ptr<Constraint> &c);
        std::shared_ptr<Constraint> &getConstraint(int id);
        bool initialize();
        bool solve(unsigned int iteration);
        std::shared_ptr<MatrixX3> getModelPoints() const;
        void setModelPoints(const std::shared_ptr<MatrixX3> &value);
        std::shared_ptr<MatrixX3> getCagePoints() const;
        void setCagePoints(const std::shared_ptr<MatrixX3> &value);
        std::shared_ptr<MatrixXX> getBarycentricCoordinates() const;
        void setBarycentricCoordinates(const std::shared_ptr<MatrixXX> &value);

    private:
        typedef std::vector<std::shared_ptr<Constraint> > Constraints;
        std::shared_ptr<MatrixX3> modelPoints;
        std::shared_ptr<MatrixX3> projections;
        std::shared_ptr<MatrixX3> cagePoints;
        std::shared_ptr<MatrixXX> barycentricCoordinates;
        Constraints constraints;
        Eigen::SimplicialLDLT<SparseMatrix> solver;

        SparseMatrix At_;
        ShapeOp::SparseMatrix Ut;
        SparseMatrix N_;
    };

}
#endif // SOLVER_H
