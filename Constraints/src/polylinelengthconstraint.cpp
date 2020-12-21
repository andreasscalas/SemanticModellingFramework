#include "polylinelengthconstraint.h"

namespace ShapeOp{
    PolylineLengthConstraint::PolylineLengthConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions, const double rangeMin, const double rangeMax) :
        Constraint(idI, weight), rangeMin(rangeMin), rangeMax(rangeMax)
    {
        assert(idI.size() >= 2);
        input = Matrix3X::Zero(3, idI.size());
        this->modelPoints = positions;
    }

    ///////////////////////////////////////////////////////////////////////////////
    void PolylineLengthConstraint::project(std::shared_ptr<MatrixX3> projections) const
    {
        for (int i = 0; i < static_cast<int>(idList.size()); ++i) input.col(i) = modelPoints->col(idList[i]);
        //Compute and subtract mean
        double l = 0;
        std::vector<Vector3> edges;
        std::vector<Vector3> newPos;
        for(unsigned int i = 1; i < idList.size(); i++){
            edges.push_back(input.col(i) - input.col(i - 1));
            l += edges[i - 1].norm();
        }

        double proportion = 1;
        if( l > rangeMax)
            proportion = rangeMax / l;
        else if (l < rangeMin)
            proportion = rangeMin / l;
        newPos.push_back(input.col(0));
        Vector3 center = newPos[0];
        for(unsigned int i = 1; i < idList.size(); i++){
            newPos.push_back(newPos[i - 1] + edges[i - 1] * proportion);
            center += newPos[i];
        }

        center /= idList.size();

        for(unsigned int i = 0; i < idList.size(); i++)
            projections->row(this->idO + i) = weight * (newPos[i] - center);


    }

    ///////////////////////////////////////////////////////////////////////////////
    void PolylineLengthConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
    {
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

    ///////////////////////////////////////////////////////////////////////////////
    double PolylineLengthConstraint::getRangeMin() const
    {
        return rangeMin;
    }

    ///////////////////////////////////////////////////////////////////////////////
     void PolylineLengthConstraint::setRangeMin(const double &value)
    {
        rangeMin = value;
    }

    ///////////////////////////////////////////////////////////////////////////////
    double PolylineLengthConstraint::getRangeMax() const
    {
        return rangeMax;
    }

    ///////////////////////////////////////////////////////////////////////////////
    void PolylineLengthConstraint::setRangeMax(const double &value)
    {
        rangeMax = value;
    }
}
