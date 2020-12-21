#include "edgestrainconstraint.h"
#include <types.h>

namespace ShapeOp {

    double clamp(double v, double vMin, double vMax) {
        double result = v > vMin ? v : vMin;
        return result > vMax ? vMax : result;
    }

    EdgeStrainConstraint::EdgeStrainConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions, double rangeMin, double rangeMax) :
        Constraint(idI, weight), rangeMin_(rangeMin), rangeMax_(rangeMax)
    {
        assert(idI.size() == 2);
        double length = (positions->col(idList[1]) - positions->col(idList[0])).norm();
        this->modelPoints = positions;
    }


    void EdgeStrainConstraint::project(std::shared_ptr<MatrixX3> projections) const {
        Vector3 edge = modelPoints->col(idList[1]) - modelPoints->col(idList[0]);
        double l = edge.norm();
        edge.normalize();
        l = clamp(l, rangeMin_, rangeMax_);
        edge *= l;
        Vector3 newPos1 = modelPoints->col(idList[0]);
        Vector3 newPos2 = modelPoints->col(idList[0]) + edge;
        Vector3 center = (modelPoints->col(idList[0]) + newPos2) / 2;
        projections->row(this->idO) = weight * (newPos1 - center);
        projections->row(this->idO + 1) = weight * (newPos2 - center);
    }

    void EdgeStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
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

    void EdgeStrainConstraint::setEdgeLength(double length)
    {
      rest_ = 1.0f/length;
    }


}
