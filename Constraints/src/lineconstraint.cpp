#include "lineconstraint.h"
#include <iostream>

namespace ShapeOp{
    LineConstraint::LineConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions) :
        Constraint(idI, weight)
    {

        assert(idI.size() >= 2);
        input = Matrix3X::Zero(3, idI.size());
        this->modelPoints = positions;
    }

    void LineConstraint::project(std::shared_ptr<MatrixX3> projections) const {
        for (int i = 0; i < static_cast<int>(idList.size()); ++i) input.col(i) = modelPoints->col(idList[i]);
        Vector3 mean_vector = input.rowwise().mean();
        input.colwise() -= mean_vector;
        Eigen::JacobiSVD<Matrix3X> jSVD;
        jSVD.compute(input, Eigen::ComputeFullU);
        Vector3 fittedLine = jSVD.matrixU().col(0);
        for (unsigned int i = 0; i < idList.size(); ++i)
            projections->row(idO + i) = (input.col(i).dot(fittedLine) / fittedLine.norm()) * fittedLine * weight;

    }

    void LineConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
        this->idO = idO;
        int n_idx = static_cast<int>(idList.size());
        double coef1 = (1.0 - 1.0 / n_idx) * weight;
        double coef2 = - weight / n_idx;
        for (int i = 0; i < n_idx; ++i) {
          for (int j = 0; j < n_idx; ++j)
            triplets.push_back(Triplet(idO, idList[j], (i == j ? coef1 : coef2)));
          idO++;
        }
    }
}
