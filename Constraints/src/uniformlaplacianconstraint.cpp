#include "uniformlaplacianconstraint.h"

namespace ShapeOp {


    UniformLaplacianConstraint::UniformLaplacianConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions, bool displacement_lap) :
        Constraint(idI, weight)
    {
        this->modelPoints = positions;
        displacement.setZero();
        if (displacement_lap) {
            int n_idx = static_cast<int>(idI.size());
            for (int i = 1; i < n_idx; ++i) displacement += positions->col(idI[i]);
            displacement /= double(n_idx - 1);
            displacement -= positions->col(idI[0]);
            displacement *= weight;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    void UniformLaplacianConstraint::project(std::shared_ptr<MatrixX3> projections) const {
        projections->row(this->idO) = displacement;
    }

    ///////////////////////////////////////////////////////////////////////////////
    void UniformLaplacianConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
        this->idO = idO;
        triplets.push_back(Triplet(this->idO, idList[0], -weight));
        double c = weight / (idList.size() - 1);
        for (int i = 1; i < static_cast<int>(idList.size()); ++i)
        triplets.push_back(Triplet(this->idO, idList[i], c));
        idO ++;
    }

    Vector3 UniformLaplacianConstraint::getDisplacement() const
    {
        return displacement;
    }

    void UniformLaplacianConstraint::setDisplacement(const Vector3 &weighted_rest)
    {
        displacement = weighted_rest;
    }

}
