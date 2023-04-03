#include "constraintsolver.h"
#include <chrono>

ConstraintSolver::ConstraintSolver()
{
    reachedConstraintID = 0;
}

ConstraintSolver::~ConstraintSolver()
{
    this->clear();
}

void ConstraintSolver::clear()
{
    for(unsigned int i = 0; i < constraints.size(); i++)
        constraints[i].reset();
    this->constraints.clear();
    this->cagePoints = nullptr;
    this->barycentricCoordinates = nullptr;
}

double ConstraintSolver::getResidualError()
{
    std::vector<double> errors = getResidualErrors();
    double totalError = 0;
    for(unsigned int i = 0; i < errors.size(); i++)
        totalError += errors[i];
    return totalError;
}

double ConstraintSolver::getResidualError(unsigned int index)
{
    double error = 0;
    constraints[index]->project(modelPointsTransposed, projectionsTransposed);
    std::vector<int> ids = constraints[index]->getIds();
    for(unsigned int j = 0; j < constraints[index]->nIndices(); j++){
        ShapeOp::Vector3 x = modelPointsTransposed.col(ids[j]);
        ShapeOp::Vector3 p = projectionsTransposed.col(static_cast<unsigned int>(constraints[index]->getIdO()) + j);
        error += (x - p).norm();
    }

    return error;
}

double ConstraintSolver::getResidualError(std::shared_ptr<ShapeOp::Constraint> constraint)
{
    double error = 0;
    constraint->project(modelPointsTransposed, projectionsTransposed);
    std::vector<int> ids = constraint->getIds();
    for(unsigned int j = 0; j < constraint->nIndices(); j++){
        ShapeOp::Vector3 x = modelPointsTransposed.col(ids[j]);
        ShapeOp::Vector3 p = projectionsTransposed.col(static_cast<unsigned int>(constraint->getIdO()) + j);
        error += (x - p).norm();
    }

    error /= constraint->nIndices();

    return error;
}

std::vector<double> ConstraintSolver::getResidualErrors()
{
    std::vector<double> errors;
    //Laplacian constraints only project one vertex of the set passed to the constraint
    for (unsigned int i = 0; i < static_cast<unsigned int>(constraints.size()); ++i){
        constraints[i]->project(modelPointsTransposed, projectionsTransposed);
        errors.push_back(0);
        std::vector<int> ids = constraints[i]->getIds();
        for(unsigned int j = 0; j < constraints[i]->nIndices(); j++){
            ShapeOp::Vector3 x = modelPointsTransposed.col(ids[j]);
            ShapeOp::Vector3 p = projectionsTransposed.col(static_cast<unsigned int>(constraints[i]->getIdO()) + j);
            errors[i] += (x - p).norm();
        }
        errors[i] /= constraints[i]->nIndices();
    }

    return errors;
}

int ConstraintSolver::addConstraint(const std::shared_ptr<ShapeOp::Constraint> &c)
{
    constraints.push_back(c);
    return static_cast<int>(constraints.size() - 1);
}

bool ConstraintSolver::removeConstraint(const unsigned int cid)
{
    if (cid >= constraints.size()) return false;
    constraints.erase(constraints.begin() + cid);
    return true;
}

bool ConstraintSolver::setConstraint(const unsigned int cid, const std::shared_ptr<ShapeOp::Constraint> &c)
{
    if(constraints.size() > cid){
        constraints[cid] = c;
        return true;
    }
    return false;
}

void ConstraintSolver::checkConstraints()
{
    for(unsigned int i = 0; i < semanticConstraints.size(); i++)
    {
        AnnotationsConstraint* c = semanticConstraints[i];
        c->checkConstraint();
    }
}

std::shared_ptr<ShapeOp::Constraint> &ConstraintSolver::getConstraint(const unsigned int cid)
{
    return constraints[cid];
}

bool ConstraintSolver::initialize()
{
    Eigen::initParallel();

    unsigned int n_points = static_cast<unsigned int>(modelPointsTransposed.cols());
    unsigned int n_constraints = static_cast<unsigned int>(constraints.size());
    if(n_points == 0 || n_constraints == 0) return false;
    std::vector<ShapeOp::Triplet> triplets;
    int idO = 0;

    for (unsigned int i = 0; i < n_constraints; ++i)
        constraints[i]->addConstraint(triplets, idO);

    projectionsTransposed.setZero(3, idO);
    ShapeOp::SparseMatrix Q;
    Q.resize(idO, n_points);
    ShapeOp::SparseMatrix U;
    U.resize(Q.rows(), barycentricCoordinates->cols());
    Q.setFromTriplets(triplets.begin(), triplets.end());
    std::cout << "Size of Q: " << Q.rows() << "x" << Q.cols() << std::endl;
    std::cout << "Size of B: " << barycentricCoordinates->rows() << "x" << barycentricCoordinates->cols() << std::endl;
    std::cout << "Number of non-zero entries: " << barycentricCoordinates->nonZeros() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    U = Q * (*barycentricCoordinates);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "First task executed in " << duration.count() << " seconds!"  << std::endl;
    start = std::chrono::high_resolution_clock::now();
    Ut = U.transpose();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Second task executed in " << duration.count() << " seconds!"  << std::endl;

    start = std::chrono::high_resolution_clock::now();
    solver = std::make_shared<ShapeOp::SimplicialLDLTSolver>();
    start = std::chrono::high_resolution_clock::now();
    ShapeOp::SparseMatrix tmp = Ut * U;
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Third task executed in " << duration.count() << " seconds!"  << std::endl;
    std::cout << "U^t*U is a " << tmp.rows() << "x" << tmp.cols() << " matrix!"  << std::endl;
    start = std::chrono::high_resolution_clock::now();
    solver->initialize(tmp);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);

    bool success = solver->info() == Eigen::Success;

    success ? std::cout << "Initialized in " << duration.count() << " seconds!" : std::cout << "Error in  initialization."<< std::endl;
    std::cout << std::endl;
    return success;
}

bool ConstraintSolver::solve(unsigned int iterationNumber)
{

    for (unsigned int it = 0; it < iterationNumber; ++it) {
        {
            #pragma omp parallel for
            //local solve: projection
            for (unsigned int i = 0; i < static_cast<unsigned int>(constraints.size()); ++i)
                constraints[i]->project(modelPointsTransposed, projectionsTransposed);

            #pragma omp parallel for
            //global solve:  merging
            for (unsigned int i = 0; i < 3; ++i){
                ShapeOp::VectorX pt = projectionsTransposed.row(i);
                cagePoints->col(i) = solver->solve(Ut * pt, cagePoints->col(i));
            }
        }
    }


    return true;

}

ShapeOp::MatrixX3 *ConstraintSolver::getCagePoints() const
{
    return cagePoints;
}

void ConstraintSolver::setCagePoints(ShapeOp::MatrixX3 *value)
{
    cagePoints = value;
}

ShapeOp::Matrix3X ConstraintSolver::getModelPointsTransposed() const
{
    return modelPointsTransposed;
}

void ConstraintSolver::setModelPointsTransposed(const ShapeOp::Matrix3X &value)
{
    modelPointsTransposed = value;
}

ShapeOp::SparseMatrix *ConstraintSolver::getBarycentricCoordinates() const
{
    return barycentricCoordinates;
}

void ConstraintSolver::setBarycentricCoordinates(ShapeOp::SparseMatrix *value)
{
    barycentricCoordinates = value;
}

int ConstraintSolver::addSemanticConstraint(AnnotationsConstraint *c)
{
    semanticConstraints.push_back(c);
    c->setId(reachedConstraintID++);
    //c->setPoints(this->modelPointsTransposed);
    return static_cast<int>(semanticConstraints.size() - 1);
}

bool ConstraintSolver::removeSemanticConstraint(const unsigned int cid)
{
    if (cid >= semanticConstraints.size()) return false;
    semanticConstraints.erase(semanticConstraints.begin() + cid);
    return true;
}

bool ConstraintSolver::setSemanticConstraint(const unsigned int cid, AnnotationsConstraint *c)
{
    if(semanticConstraints.size() > cid){
        semanticConstraints[cid] = c;
        return true;
    }
    return false;
}

AnnotationsConstraint *ConstraintSolver::getSemanticConstraint(const unsigned int cid)
{
    return semanticConstraints[cid];
}
