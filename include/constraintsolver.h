#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

#include <vector>
#include <memory>
#include <Constraint.h>
#include <Types.h>
#include <LSSolver.h>
#include <infopalette.h>
class ConstraintSolver
{
public:

    typedef std::vector<std::shared_ptr<ShapeOp::Constraint> > Constraints;

    ConstraintSolver();
    ~ConstraintSolver();

    void clear();

    double getResidualError();
    double getResidualError(unsigned int);
    double getResidualError(std::shared_ptr<ShapeOp::Constraint>);
    std::vector<double> getResidualErrors();

    int addConstraint(const std::shared_ptr<ShapeOp::Constraint>& c);

    bool removeConstraint(const unsigned int cid);

    bool setConstraint(const unsigned int cid, const std::shared_ptr<ShapeOp::Constraint> &c);

    std::shared_ptr<ShapeOp::Constraint>& getConstraint(const unsigned int cid);

    bool initialize();

    bool solve(unsigned int iterationNumber);

    ShapeOp::MatrixX3 *getCagePoints() const;
    void setCagePoints(ShapeOp::MatrixX3 *value);


    ShapeOp::Matrix3X getModelPointsTransposed() const;
    void setModelPointsTransposed(const ShapeOp::Matrix3X &value);

    ShapeOp::SparseMatrix *getBarycentricCoordinates() const;
    void setBarycentricCoordinates(ShapeOp::SparseMatrix *value);

private:
    ShapeOp::Matrix3X modelPointsTransposed;
    ShapeOp::MatrixX3* cagePoints;
    ShapeOp::SparseMatrix* barycentricCoordinates;
    ShapeOp::Matrix3X projectionsTransposed;
    ShapeOp::SparseMatrix Ut;
    Constraints constraints;
    std::shared_ptr<ShapeOp::LSSolver> solver;
    ShapeOp::SparseMatrix Q;


};

#endif // CONSTRAINTSOLVER_H
