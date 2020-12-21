#include "closenessconstraint.h"
#include <iostream>
namespace ShapeOp{

ClosenessConstraint::ClosenessConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions) :
    Constraint(idI, weight) {
    assert(idI.size() == 1);
    position = positions->col(idList[0]);
    this->modelPoints = positions;
}

void ClosenessConstraint::project(std::shared_ptr<MatrixX3> projections) const {
    projections->row(this->idO) = position * weight;
}

void ClosenessConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    this->idO = idO;
    triplets.push_back(Triplet(this->idO, idList[0], weight));
    idO += 1;
}

void ClosenessConstraint::setPosition(const Vector3 &position) {
    this->position = position;
}

Vector3 ClosenessConstraint::getPosition() const {
    return position;
}

}
