#include <solver.h>
#include <iostream>
namespace ShapeOp{

int Solver::addConstraint(const std::shared_ptr<Constraint> &c){
    constraints.push_back(c);
    return static_cast<int>(constraints.size() - 1);
}

bool Solver::initialize()
{
    unsigned int n_points = static_cast<unsigned int>(modelPoints->rows());
    unsigned int n_constraints = static_cast<unsigned int>(constraints.size());
    if(n_points == 0 || n_constraints == 0) return false;
    std::vector<ShapeOp::Triplet> triplets;
    int idO = 0;

    for (unsigned int i = 0; i < n_constraints; ++i)
        constraints[i]->addConstraint(triplets, idO);

    projections = std::make_shared<MatrixX3>();
    projections->setZero(idO, 3);
    ShapeOp::SparseMatrix Q(idO, n_points);
    ShapeOp::SparseMatrix U;
    Q.setFromTriplets(triplets.begin(), triplets.end());
    U = (Q * (*barycentricCoordinates)).sparseView(1e-5);
    Ut = U.transpose();
    ShapeOp::SparseMatrix tmp = Ut * U;
    solver.compute(tmp);
    return (solver.info() == Eigen::Success);
}

bool Solver::solve(unsigned int iterationNumber){
    for (unsigned int it = 0; it < iterationNumber; ++it) {
        {
            //local solve: projection
            for (unsigned int i = 0; i < static_cast<unsigned int>(constraints.size()); ++i)
                constraints[i]->project(projections);

            //global solve:  merging
            for (unsigned int i = 0; i < 3; ++i){
                ShapeOp::VectorX pt = projections->col(i);
                ShapeOp::VectorX tmp = Ut * pt;
                if(solver.info() == Eigen::Success)
                    cagePoints->col(i) = solver.solve(tmp);
                else
                    return false;
            }
        }
    }
    return true;
}

std::shared_ptr<MatrixX3> Solver::getModelPoints() const
{
    return modelPoints;
}

void Solver::setModelPoints(const std::shared_ptr<MatrixX3> &value)
{
    modelPoints = value;
}

std::shared_ptr<MatrixX3> Solver::getCagePoints() const
{
    return cagePoints;
}

void Solver::setCagePoints(const std::shared_ptr<MatrixX3> &value)
{
    cagePoints = value;
}

std::shared_ptr<MatrixXX> Solver::getBarycentricCoordinates() const
{
    return barycentricCoordinates;
}

void Solver::setBarycentricCoordinates(const std::shared_ptr<MatrixXX> &value)
{
    barycentricCoordinates = value;
}


}
