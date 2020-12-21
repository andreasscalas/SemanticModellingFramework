#include "orientationconstraint.h"

namespace ShapeOp {

    OrientationConstraint::OrientationConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> &positions, Vector3 normal /*= Vector3(0.0,0.0,1.0)*/):
        Constraint(idI,weight)
    {
        assert(idI.size() >= 3);
        setOrientation(normal);
        //Allocate memory for intermediate storage during projection
        input = Matrix3X::Zero(3, idI.size());
        this->modelPoints = positions;
    }

    void OrientationConstraint::setOrientation(const Vector3 &normal){
        this->normal = normal;
        this->normal.normalize();
    }


///////////////////////////////////////////////////////////////////////////////
    void OrientationConstraint::project(std::shared_ptr<MatrixX3> projections) const {
        //Copy the constrained positions to input
        for (int i = 0; i < static_cast<int>(idList.size()); ++i) input.col(i) = modelPoints->col(idList[i]);
        //Compute and subtract mean
        Vector3 mean_vector = input.rowwise().mean();
        input.colwise() -= mean_vector;
        Matrix33 U = input.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
        double angle = std::acos(U.col(2).dot(normal) / U.col(2).norm());
        Vector3 rotAxis = U.col(2).cross(normal);
        rotAxis.normalize();
        Matrix33 rotMatrix;
        rotMatrix(0, 0) = cos(angle) + pow(rotAxis(0), 2) * (1 - cos(angle));
        rotMatrix(0, 1) = rotAxis(0) * rotAxis(1) * (1 - cos(angle)) - rotAxis(2) * sin(angle);
        rotMatrix(0, 2) = rotAxis(0) * rotAxis(2) * (1 - cos(angle)) + rotAxis(1) * sin(angle);
        rotMatrix(1, 0) = rotAxis(1) * rotAxis(0) * (1 - cos(angle)) + rotAxis(2) * sin(angle);
        rotMatrix(1, 1) = cos(angle) + pow(rotAxis(1), 2) * (1 - cos(angle));
        rotMatrix(1, 2) = rotAxis(1) * rotAxis(2) * (1 - cos(angle)) - rotAxis(0) * sin(angle);
        rotMatrix(2, 0) = rotAxis(2) * rotAxis(0) * (1 - cos(angle)) - rotAxis(1) * sin(angle);
        rotMatrix(2, 1) = rotAxis(2) * rotAxis(1) * (1 - cos(angle)) + rotAxis(0) * sin(angle);
        rotMatrix(2, 2) = cos(angle) + pow(rotAxis(2), 2) * (1 - cos(angle));

        for (int i = 0; i < static_cast<int>(idList.size()); ++i)
            projections->row(this->idO + i) = (rotMatrix * input.col(i)) * weight;

    }


///////////////////////////////////////////////////////////////////////////////
    void OrientationConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
        //Store this constraints position in the global linear system
        this->idO = idO;
        //Precompute coefficients for mean-centering
        int n_idx = static_cast<int>(idList.size());
        double coef1 = (1.0 - 1.0 / n_idx) * weight;
        double coef2 = -weight / n_idx;
        //Add triplets to the sparse global linear system.
        for (int i = 0; i < n_idx; ++i) {
        for (int j = 0; j < n_idx; ++j)
          //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idList[j].
          triplets.push_back(Triplet(idO, idList[j], (i == j ? coef1 : coef2)));
        idO++;
        }
    }

}
