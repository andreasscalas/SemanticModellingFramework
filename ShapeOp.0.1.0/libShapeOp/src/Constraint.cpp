///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014-2015 Sofien Bouaziz <sofien.bouaziz@gmail.com>, Bailin Deng <bldeng@gmail.com>, Mario Deuss <mario.deuss@epfl.ch>
// Copyright (C) 2014-2015 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#include "Constraint.h"
#include <cassert>
#include <algorithm>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <float.h>
#include <set>
///////////////////////////////////////////////////////////////////////////////
#define SHAPEOP_INNER_ITERATIONS 4 //TODO: fix this
#define DEBUG_MODE true
///////////////////////////////////////////////////////////////////////////////
namespace ShapeOp {
///////////////////////////////////////////////////////////////////////////////
/** \brief Clamps v to lie between vMin and vMax.*/
Scalar clamp(Scalar v, Scalar vMin, Scalar vMax) {
  Scalar result = v > vMin ? v : vMin;
  return result > vMax ? vMax : result;
}


Matrix33 createRotationMatrix(Vector3 rotationAxis, Scalar angle){

    Matrix33 rotationMatrix;
    rotationMatrix(0, 0) = cos(angle) + pow(rotationAxis(0), 2) * (1 - cos(angle));
    rotationMatrix(0, 1) = rotationAxis(0) * rotationAxis(1) * (1 - cos(angle)) - rotationAxis(2) * sin(angle);
    rotationMatrix(0, 2) = rotationAxis(0) * rotationAxis(2) * (1 - cos(angle)) + rotationAxis(1) * sin(angle);
    rotationMatrix(1, 0) = rotationAxis(0) * rotationAxis(1) * (1 - cos(angle)) + rotationAxis(2) * sin(angle);
    rotationMatrix(1, 1) = cos(angle) + pow(rotationAxis(1), 2) * (1 - cos(angle));
    rotationMatrix(1, 2) = rotationAxis(1) * rotationAxis(2) * (1 - cos(angle)) - rotationAxis(0) * sin(angle);
    rotationMatrix(2, 0) = rotationAxis(0) * rotationAxis(2) * (1 - cos(angle)) - rotationAxis(1) * sin(angle);
    rotationMatrix(2, 1) = rotationAxis(1) * rotationAxis(2) * (1 - cos(angle)) + rotationAxis(0) * sin(angle);
    rotationMatrix(2, 2) = cos(angle) + pow(rotationAxis(2), 2) * (1 - cos(angle));

    return rotationMatrix;

}

///////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Constraint> Constraint::shapeConstraintFactory(const std::string &constraintType, const std::vector<int> &idI, Scalar weight, const Matrix3X &positions) {
    std::size_t n = idI.size();
    std::shared_ptr<Constraint> c;
    if (constraintType.compare("EdgeStrain") == 0) {                if (n != 2) { return c; } return std::make_shared<EdgeStrainConstraint>(idI, weight, positions); }
    if (constraintType.compare("TriangleStrain") == 0) {            if (n != 3) { return c; } return std::make_shared<TriangleStrainConstraint>(idI, weight, positions); }
    if (constraintType.compare("TetrahedronStrain") == 0) {         if (n != 4) { return c; } return std::make_shared<TetrahedronStrainConstraint>(idI, weight, positions); }
    if (constraintType.compare("Area") == 0) {                      if (n != 3) { return c; } return std::make_shared<AreaConstraint>(idI, weight, positions); }
    if (constraintType.compare("Volume") == 0) {                    if (n != 4) { return c; } return std::make_shared<VolumeConstraint>(idI, weight, positions); }
    if (constraintType.compare("Bending") == 0) {                   if (n != 4) { return c; } return std::make_shared<BendingConstraint>(idI, weight, positions); }
    if (constraintType.compare("Closeness") == 0) {                 if (n != 1) { return c; } return std::make_shared<ClosenessConstraint>(idI, weight, positions); }
    if (constraintType.compare("Line") == 0) {                      if (n <  2) { return c; } return std::make_shared<LineConstraint>(idI, weight, positions); }
    if (constraintType.compare("Plane") == 0) {                     if (n <  3) { return c; } return std::make_shared<PlaneConstraint>(idI, weight, positions); }
    if (constraintType.compare("Circle") == 0) {                    if (n <  3) { return c; } return std::make_shared<CircleConstraint>(idI, weight, positions); }
    if (constraintType.compare("Sphere") == 0) {                    if (n <  4) { return c; } return std::make_shared<SphereConstraint>(idI, weight, positions); }
    if (constraintType.compare("Similarity") == 0) {                if (n <  1) { return c; } return std::make_shared<SimilarityConstraint>(idI, weight, positions, true); }
    if (constraintType.compare("Rigid") == 0) {                     if (n <  1) { return c; } return std::make_shared<SimilarityConstraint>(idI, weight, positions, false); }
    if (constraintType.compare("Rectangle") == 0) {                 if (n != 4) { return c; } return std::make_shared<RectangleConstraint>(idI, weight, positions); }
    if (constraintType.compare("Parallelogram") == 0) {             if (n != 4) { return c; } return std::make_shared<ParallelogramConstraint>(idI, weight, positions); }
    if (constraintType.compare("Laplacian") == 0) {                 if (n <  2) { return c; } return std::make_shared<UniformLaplacianConstraint>(idI, weight, positions, false); }
    if (constraintType.compare("LaplacianDisplacement") == 0) {     if (n <  2) { return c; } return std::make_shared<UniformLaplacianConstraint>(idI, weight, positions, true); }
    if (constraintType.compare("CotangentLaplacian") == 0) {                 if (n <  2) { return c; } return std::make_shared<CotangentLaplacianConstraint>(idI, weight, positions, false); }
    if (constraintType.compare("CotangentLaplacianDisplacement") == 0) {     if (n <  2) { return c; } return std::make_shared<CotangentLaplacianConstraint>(idI, weight, positions, true); }
    if (constraintType.compare("Angle") == 0) {                     if (n != 3) { return c; } return std::make_shared<AngleConstraint>(idI, weight, positions); }
    if (constraintType.compare("Orientation") == 0){                if (n <  1) { return c; } return std::make_shared<OrientationConstraint>(idI, weight, positions); }
    if (constraintType.compare("RegressionPlaneOrientation") == 0){ if (n <  3) { return c; } return std::make_shared<RegressionPlaneOrientationConstraint>(idI, weight, positions); }
    if (constraintType.compare("PolylineLength") == 0){             if (n <  3) { return c; } return std::make_shared<PolylineLengthConstraint>(idI, weight, positions, 0, 1); }
    if (constraintType.compare("RectangleFit") == 0){               if (n <  4) { return c; } return std::make_shared<RectangleFitConstraint>(idI, weight, positions); }
    if (constraintType.compare("Equilateral") == 0){                    if (n != 3) { return c; } return std::make_shared<EquilateralConstraint>(idI, weight, positions); }
    if (constraintType.compare("SimilarLength") == 0){              return std::make_shared<SimilarLengthConstraint>(idI, weight, positions); }
    if (constraintType.compare("SameMeasure") == 0){                return std::make_shared<NonUniformScalingConstraint>(idI, weight, positions); }
    if (constraintType.compare("Coaxiality") == 0){                 return std::make_shared<CoaxialityConstraint>(idI, weight, positions); }

    return c;
}
///////////////////////////////////////////////////////////////////////////////
Constraint::Constraint(const std::vector<int> &idI, Scalar weight) :
  idI_(idI),
  weight_(std::sqrt(weight)) {
}

Scalar Constraint::getWeight() const
{
    return weight_;
}

void Constraint::setWeight(const Scalar &weight)
{
    weight_ = weight;
}

int Constraint::getIdO() const
{
    return idO_;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
EdgeStrainConstraint::EdgeStrainConstraint(const std::vector<int> &idI,
                                                          Scalar weight,
                                                          const Matrix3X &positions,
                                                          Scalar rangeMin,
                                                          Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 2);
  Scalar length = (positions.col(idI_[1]) - positions.col(idI_[0])).norm();
}
///////////////////////////////////////////////////////////////////////////////
void EdgeStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {

    Matrix3X projection = getProjection(positions);
    projections.col(idO_) = weight_ * projection.col(0);
    projections.col(idO_ + 1) = weight_ * projection.col(1);
}
///////////////////////////////////////////////////////////////////////////////
void EdgeStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    idO_ = idO;
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = - weight_ / nIndices();
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }
}
///////////////////////////////////////////////////////////////////////////////
Matrix3X EdgeStrainConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, 3);
    Vector3 edge = positions.col(idI_[1]) - positions.col(idI_[0]);
    Scalar l = edge.norm();
    edge.normalize();
    l = clamp(l, rangeMin_, rangeMax_);
    edge *= l;
    Vector3 newPos1 = positions.col(idI_[0]);
    Vector3 newPos2 = positions.col(idI_[0]) + edge;
    Vector3 center = (positions.col(idI_[0]) + newPos2) / 2;
    projection.col(0) = newPos1 - center;
    projection.col(1) = newPos2 - center;

    return projection;
}

void EdgeStrainConstraint::setEdgeLength(Scalar length)
{
    rest_ = 1.0f/length;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
TriangleStrainConstraint::TriangleStrainConstraint(const std::vector<int> &idI,
                                                                  Scalar weight,
                                                                  const Matrix3X &positions,
                                                                  Scalar rangeMin,
                                                                  Scalar rangeMax) :
    Constraint(idI, weight),
    rangeMin_(rangeMin),
    rangeMax_(rangeMax)
{
    assert(idI.size() == 3);
    Matrix32 edges, P;
    edges.col(0) = positions.col(idI_[1]) - positions.col(idI_[0]);
    edges.col(1) = positions.col(idI_[2]) - positions.col(idI_[0]);
    P.col(0) = edges.col(0).normalized();
    P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
    rest_ = (P.transpose() * edges).inverse();
    Scalar A = (P.transpose() * edges).determinant() / 2.0f;
    weight_ *= std::sqrt(std::abs(A));
}
///////////////////////////////////////////////////////////////////////////////
void TriangleStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    projections.block<3, 2>(0, idO_) = (weight_ * projection);
}
///////////////////////////////////////////////////////////////////////////////
void TriangleStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 2;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
  }
  idO += n;
}

Matrix3X TriangleStrainConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, nIndices());
    Matrix32 edges, P;
    edges.col(0) = (positions.col(idI_[1]) - positions.col(idI_[0]));
    edges.col(1) = (positions.col(idI_[2]) - positions.col(idI_[0]));
    P.col(0) = edges.col(0).normalized();
    P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
    Matrix22 F = P.transpose() * edges * rest_;
    Eigen::JacobiSVD<Matrix22> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector2 S = svd.singularValues();
    S(0) = clamp(S(0), rangeMin_, rangeMax_);
    S(1) = clamp(S(1), rangeMin_, rangeMax_);
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    projection = P * F;

    return projection;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
TetrahedronStrainConstraint::TetrahedronStrainConstraint(const std::vector<int> &idI,
                                                                        Scalar weight,
                                                                        const Matrix3X &positions,
                                                                        Scalar rangeMin,
                                                                        Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 4);
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  rest_ = edges.inverse();
  Scalar V = (edges).determinant() / 6.0f;
  weight_ *= std::sqrt(std::abs(V));
}
///////////////////////////////////////////////////////////////////////////////
void TetrahedronStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {

    Matrix3X projection = getProjection(positions);
    projections.block<3, 3>(0, idO_) = weight_ * projection;
}
///////////////////////////////////////////////////////////////////////////////
void TetrahedronStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 3;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i) + rest_(2, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[3], weight_ * rest_(2, i)));
  }
  idO += n;
}

Matrix3X TetrahedronStrainConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, nIndices());
    Matrix33 edges;
    for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
    Matrix33 F = edges * rest_;
    Eigen::JacobiSVD<Matrix33> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector3 S = svd.singularValues();
    S(0) = clamp(S(0), rangeMin_, rangeMax_);
    S(1) = clamp(S(1), rangeMin_, rangeMax_);
    S(2) = clamp(S(2), rangeMin_, rangeMax_);
    if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0f) S(2) = -S(2);
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    projection = F;


    return projection;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
AreaConstraint::AreaConstraint(const std::vector<int> &idI,
                                              Scalar weight,
                                              const Matrix3X &positions,
                                              Scalar rangeMin,
                                              Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 3);
  Matrix32 edges, P;
  edges.col(0) = positions.col(idI_[1]) - positions.col(idI_[0]);
  edges.col(1) = positions.col(idI_[2]) - positions.col(idI_[0]);
  P.col(0) = edges.col(0).normalized();
  P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
  rest_ = (P.transpose() * edges).inverse();
  Scalar A = (P.transpose() * edges).determinant() / 2.0f;
  weight_ *= std::sqrt(std::abs(A));
}
///////////////////////////////////////////////////////////////////////////////
void AreaConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block<3, 2>(0, idO_) = (weight_ * projection);
}
///////////////////////////////////////////////////////////////////////////////
void AreaConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 2;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
  }
  idO += n;
}

Matrix3X AreaConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, nIndices());
    Matrix32 edges, P;
    edges.col(0) = (positions.col(idI_[1]) - positions.col(idI_[0]));
    edges.col(1) = (positions.col(idI_[2]) - positions.col(idI_[0]));
    P.col(0) = edges.col(0).normalized();
    P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
    Matrix22 F = P.transpose() * edges * rest_;
    Eigen::JacobiSVD<Matrix22> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector2 S = svd.singularValues();
    Vector2 d(0.0f, 0.0f);
    for (int i = 0; i < SHAPEOP_INNER_ITERATIONS; ++i) {
      Scalar v = S(0) * S(1);
      Scalar f = v - clamp(v, rangeMin_, rangeMax_);
      Vector2 g(S(1), S(0));
      d = -((f - g.dot(d)) / g.dot(g)) * g;
      S = svd.singularValues() + d;
    }
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    projection = P * F;


    return projection;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
VolumeConstraint::VolumeConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions,
                                                  Scalar rangeMin,
                                                  Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI_.size() == 4);
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  rest_ = edges.inverse();
  Scalar V = (edges).determinant() / 6.0f;
  weight_ *= std::sqrt(std::abs(V));
}
///////////////////////////////////////////////////////////////////////////////
void VolumeConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block<3, 3>(0, idO_) = weight_ * projection;
}
///////////////////////////////////////////////////////////////////////////////
void VolumeConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    idO_ = idO;
    int n = 3;
    for (int i = 0; i < n; ++i) {
        triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i) + rest_(2, i))));
        triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
        triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
        triplets.push_back(Triplet(idO_ + i, idI_[3], weight_ * rest_(2, i)));
    }
    idO += n;
}

Matrix3X VolumeConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());
    Matrix33 edges;
    for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
    Matrix33 F = edges * rest_;
    Eigen::JacobiSVD<Matrix33> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector3 S = svd.singularValues();
    Vector3 d(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < SHAPEOP_INNER_ITERATIONS; ++i) {
      Scalar v = S(0) * S(1) * S(2);
      Scalar f = v - clamp(v, rangeMin_, rangeMax_);
      Vector3 g(S(1)*S(2), S(0)*S(2), S(0)*S(1));
      d = -((f - g.dot(d)) / g.dot(g)) * g;
      S = svd.singularValues() + d;
    }
    if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0f) S(2) = -S(2);
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    projection = F;

    return projection;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
BendingConstraint::BendingConstraint(const std::vector<int> &idI,
                                                    Scalar weight,
                                                    const Matrix3X &positions,
                                                    Scalar rangeMin,
                                                    Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  Matrix3X p(3, idI.size());
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) p.col(i) = positions.col(idI_[i]);
  Scalar l01 = (p.col(0) - p.col(1)).norm();
  Scalar l02 = (p.col(0) - p.col(2)).norm();
  Scalar l12 = (p.col(1) - p.col(2)).norm();
  Scalar r0 = 0.5 * (l01 + l02 + l12);
  Scalar A0 = std::sqrt(r0 * (r0 - l01) * (r0 - l02) * (r0 - l12));
  Scalar l03 = (p.col(0) - p.col(3)).norm();
  Scalar l13 = (p.col(1) - p.col(3)).norm();
  Scalar r1 = 0.5 * (l01 + l03 + l13);
  Scalar A1 = std::sqrt(r1 * (r1 - l01) * (r1 - l03) * (r1 - l13));
  weight_ *= std::sqrt(3.0 / (A0 + A1));
  Scalar cot02 = ((l01 * l01) - (l02 * l02) + (l12 * l12)) / (4.0 * A0);
  Scalar cot12 = ((l01 * l01) + (l02 * l02) - (l12 * l12)) / (4.0 * A0);
  Scalar cot03 = ((l01 * l01) - (l03 * l03) + (l13 * l13)) / (4.0 * A1);
  Scalar cot13 = ((l01 * l01) + (l03 * l03) - (l13 * l13)) / (4.0 * A1);
  w_ =  Vector4::Zero();
  w_(0) = cot02 + cot03;
  w_(1) = cot12 + cot13;
  w_(2) = -(cot02 + cot12);
  w_(3) = -(cot03 + cot13);
  n_ = (p * w_).norm();
}
///////////////////////////////////////////////////////////////////////////////
void BendingConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.col(idO_) = weight_ * projection;
}
///////////////////////////////////////////////////////////////////////////////
void BendingConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
    triplets.push_back(Triplet(idO_, idI_[i], weight_ * w_(i)));
  idO += 1;
}

Matrix3X BendingConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, 1);
    Vector3 e = Vector3::Zero();
    if (n_ > 1e-6) {
      for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
        e += w_(i) * positions.col(idI_[i]);
      Scalar l = e.norm();
      if (l > 1e-6) {
        e /= l;
        l = n_ * clamp(l / n_, rangeMin_, rangeMax_);
        e *= l;
      }
    }
    projection.col(0) = e;

    return projection;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
ClosenessConstraint::ClosenessConstraint(const std::vector<int> &idI,
                                                        Scalar weight,
                                                        const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 1);
  rest_ = positions.col(idI_[0]);
}
///////////////////////////////////////////////////////////////////////////////
void ClosenessConstraint::project(const Matrix3X & /*positions*/, Matrix3X &projections) const {
    projections.col(idO_) = rest_ * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void ClosenessConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    idO_ = idO;
    triplets.push_back(Triplet(idO_, idI_[0], weight_));
    idO += 1;
}

Matrix3X ClosenessConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, 1);
    projection.col(0) = rest_;

    return projection;
}
///////////////////////////////////////////////////////////////////////////////
void ClosenessConstraint::setPosition(const Vector3 &position) {
    rest_ = position;
}
///////////////////////////////////////////////////////////////////////////////
Vector3 ClosenessConstraint::getPosition() const {
    return rest_;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
LineConstraint::LineConstraint(const std::vector<int> &idI,
                                              Scalar weight,
                                              const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 2);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
void LineConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void LineConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X LineConstraint::getProjection(const Matrix3X &positions) const
{
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Eigen::JacobiSVD<Matrix3X> jSVD;
    jSVD.compute(input, Eigen::ComputeFullU);
    Matrix33 basis = jSVD.matrixU();
    input = basis.transpose() * input;
    input.row(1) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    return basis * input;

}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
PlaneConstraint::PlaneConstraint(const std::vector<int> &idI,
                                                Scalar weight,
                                                const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 3);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
void PlaneConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void PlaneConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X PlaneConstraint::getProjection(const Matrix3X &positions) const
{
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Eigen::JacobiSVD<Matrix3X> jSVD;
    jSVD.compute(input, Eigen::ComputeFullU);
    Matrix33 basis = jSVD.matrixU();
    input = basis.transpose() * input;
    input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    return basis * input;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
CircleConstraint::CircleConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 3);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
void CircleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);

    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void CircleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X CircleConstraint::getProjection(const Matrix3X &positions) const
{
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Eigen::JacobiSVD<Matrix3X> jSVD;
    jSVD.compute(input, Eigen::ComputeFullU);
    Matrix33 basis = jSVD.matrixU();
    input = basis.transpose() * input;
    input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    ////////////// 2D Circle fitting
    double Suu = 0.0;
    double Suv = 0.0;
    double Svv = 0.0;
    double Suuu = 0.0;
    double Suvv = 0.0;
    double Svuu = 0.0;
    double Svvv = 0.0;
    for (int j = 0; j < input.cols(); ++j) {
        double uu = input(0, j) * input(0, j);
        double vv = input(1, j) * input(1, j);
        Suu += uu;
        Svv += vv;
        Suv += input(0, j) * input(1, j);
        Suuu += uu * input(0, j);
        Suvv += input(0, j) * vv;
        Svuu += input(1, j) * uu;
        Svvv += vv * input(1, j);
    }
    Matrix22 A;
    A << Suu, Suv,  Suv, Svv;
    if (std::fabs(A.determinant()) > 1e-5) {
        Vector2 b(0.5 * (Suuu + Suvv), 0.5 * (Svvv + Svuu));
        Vector2 center = A.inverse() * b;
        double radius = std::sqrt(center(0) * center(0) + center(1) * center(1) + (Suu + Svv) / static_cast<double>(input.cols()));
        for (int j = 0; j < input.cols(); ++j) {
            Vector2 d = input.block(0, j, 2, 1) - center;
            d.normalize();
            input.block(0, j, 2, 1) = center + d * radius;
        }
    }
    return basis * input;

}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SphereConstraint::SphereConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 4);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
void SphereConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void SphereConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X SphereConstraint::getProjection(const Matrix3X &positions) const
{
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    ////////////// 3D Sphere fitting
    double Suu = 0.0;
    double Suv = 0.0;
    double Suw = 0.0;
    double Svv = 0.0;
    double Svw = 0.0;
    double Sww = 0.0;
    double Suuu = 0.0;
    double Suvv = 0.0;
    double Suww = 0.0;
    double Svuu = 0.0;
    double Svvv = 0.0;
    double Svww = 0.0;
    double Swuu = 0.0;
    double Swvv = 0.0;
    double Swww = 0.0;
    for (int j = 0; j < input.cols(); ++j) {
        double uu = input(0, j) * input(0, j);
        double vv = input(1, j) * input(1, j);
        double ww = input(2, j) * input(2, j);
        Suu += uu;
        Svv += vv;
        Sww += ww;
        Suv += input(0, j) * input(1, j);
        Suw += input(0, j) * input(2, j);
        Svw += input(1, j) * input(2, j);
        Suuu += input(0, j) * uu;
        Suvv += input(0, j) * vv;
        Suww += input(0, j) * ww;
        Svuu += input(1, j) * uu;
        Svvv += input(1, j) * vv;
        Svww += input(1, j) * ww;
        Swuu += input(2, j) * uu;
        Swvv += input(2, j) * vv;
        Swww += input(2, j) * ww;
    }
    Matrix33 A;
    A << Suu, Suv, Suw,  Suv, Svv, Svw, Suw, Svw, Sww;
    if (std::fabs(A.determinant()) > 1e-5) {
        Vector3 b(0.5 * (Suuu + Suvv + Suww), 0.5 * (Svuu + Svvv + Svww), 0.5 * (Swuu + Swvv + Swww));
        Vector3 center = A.inverse() * b;
        double radius = std::sqrt(center(0) * center(0) + center(1) * center(1) + center(2) * center(2) + (Suu + Svv + Sww) / static_cast<double>(input.cols()));
        for (int j = 0; j < input.cols(); ++j) {
            Vector3 d = input.col(j) - center;
            d.normalize();
            input.col(j) = center + d * radius;
        }
    }

    return  input;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SimilarityConstraint::SimilarityConstraint(const std::vector<int> &idI,
                                                          Scalar weight,
                                                          const Matrix3X &positions,
                                                          bool scaling /*= true*/,
                                                          bool rotate /*=true*/,
                                                          bool flip /*=true*/) :

  Constraint(idI, weight), scaling_(scaling), rotate_(rotate), flip_(flip) {
  assert(idI.size() >= 2);
  input = Matrix3X::Zero(3, idI.size());
  candidate = Matrix3X::Zero(3, idI.size());
  output = Matrix3X::Zero(3, idI.size());
  Matrix3X shape = Matrix3X::Zero(3, idI.size());
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) shape.col(i) = positions.col(idI_[i]);
  std::vector<Matrix3X> shapes;
  shapes.push_back(shape);
  setShapes(shapes);

  permutation_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  rotateMatrix_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  rotateMatrix_.indices().coeffRef(0) = static_cast<int>(idI.size()) - 1;
  for (int i = 1; i < static_cast<int>(idI.size()); ++i)
    rotateMatrix_.indices().coeffRef(i) = i - 1;
  flipMatrix_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  for (int i = 0; i < static_cast<int>(idI.size()); ++i)
    flipMatrix_.indices().coeffRef(i) = static_cast<int>(idI.size()) - 1 - i;
}
///////////////////////////////////////////////////////////////////////////////
void SimilarityConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void SimilarityConstraint::testCandidates(Scalar &min_error) const {
  for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
//      Matrix44 T = Eigen::umeyama(shapes_[i] * permutation_, input, scaling_);
//    candidate = T.block<3, 3>(0, 0) * shapes_[i] * permutation_;

      candidate = Eigen::umeyama(shapes_[i] * permutation_, input, scaling_).block<3, 3>(0, 0) * shapes_[i] * permutation_;
    Scalar error = (input - candidate).squaredNorm();
    if (error < min_error) {
      //std::cout << T << std::endl << std::endl << std::flush;
      min_error = error;
      output = candidate;
    }
  }
}

bool SimilarityConstraint::isNotRigid() const
{
    return scaling_;
}

void SimilarityConstraint::setIsNotRigid(bool scaling)
{
    scaling_ = scaling;
}
///////////////////////////////////////////////////////////////////////////////
void SimilarityConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = -weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X SimilarityConstraint::getProjection(const Matrix3X &positions) const
{
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;

    Scalar min_error = std::numeric_limits<Scalar>::max();
    permutation_.setIdentity();
    testCandidates(min_error);
    if (rotate_) {
        for (int i = 0; i < input.cols() - 1; ++i) {
            permutation_ = permutation_ * rotateMatrix_; //Rotate
            testCandidates(min_error);
        }
    }

    if (flip_) {
        permutation_.setIdentity();
        permutation_ = permutation_ * flipMatrix_; //Flip
        testCandidates(min_error);
        if (rotate_) {
            for (int i = 0; i < input.cols() - 1; ++i) {
                 permutation_ = permutation_ * rotateMatrix_; //Rotate
                 testCandidates(min_error);
            }
        }
    }
    //std::cout << "End projection" << std::endl;
    return output;
}
///////////////////////////////////////////////////////////////////////////////
void SimilarityConstraint::setShapes(const std::vector<Matrix3X> &shapes) {
  shapes_ = shapes;
  for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
    Vector3 mean_vector = shapes_[i].rowwise().mean();
    shapes_[i].colwise() -= mean_vector;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
RectangleConstraint::RectangleConstraint(const std::vector<int> &idI,
                                                        Scalar weight,
                                                        const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 4);
}
///////////////////////////////////////////////////////////////////////////////
void RectangleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block<3, 4>(0, idO_) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void RectangleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X RectangleConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix34 input;
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Eigen::JacobiSVD<Matrix3X> jSVD;
    jSVD.compute(input, Eigen::ComputeFullU);
    Matrix33 basis = jSVD.matrixU();
    input = basis.transpose() * input;
    input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    //Rectangular Fit
    MatrixT<8, 6> A;
    A << 1.0, 0.0, 0.0, 0.0, input(0, 0), input(1, 0),
    1.0, 0.0, 0.0, 0.0, input(0, 1), input(1, 1),
    0.0, 1.0, 0.0, 0.0, input(1, 1), -input(0, 1),
    0.0, 1.0, 0.0, 0.0, input(1, 2), -input(0, 2),
    0.0, 0.0, 1.0, 0.0, input(0, 2), input(1, 2),
    0.0, 0.0, 1.0, 0.0, input(0, 3), input(1, 3),
    0.0, 0.0, 0.0, 1.0, input(1, 3), -input(0, 3),
    0.0, 0.0, 0.0, 1.0, input(1, 0), -input(0, 0);
    MatrixT<8, 6> R = A.householderQr().matrixQR().triangularView<Eigen::Upper>();
    Eigen::JacobiSVD<Matrix22> jSVD2D;
    jSVD2D.compute(R.block<2, 2>(4, 4), Eigen::ComputeFullV);
    Matrix22 vec2d = jSVD2D.matrixV();
    Vector2 normal = vec2d.block<2, 1>(0, 1);
    Vector4 center = -R.block<4, 4>(0, 0).inverse() * (R.block<4, 2>(0, 4) * normal);
    //Compute the 4 corner
    Matrix22 B;
    B << normal(0), -normal(1), normal(1), normal(0);
    MatrixT<2, 4> C;
    C << center(0), center(0), center(2), center(2),
    center(3), center(1), center(1), center(3);
    MatrixT<2, 4> corners = -B * C;
    ////////
    input.block<2, 4>(0, 0) = corners;
    return basis * input;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
ParallelogramConstraint::ParallelogramConstraint(const std::vector<int> &idI,
                                                                Scalar weight,
                                                                const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 4);
  MatrixT<8, 6> H;
  H(0, 0) = 1.0; H(0, 1) = 0.0; H(0, 2) = 0.0; H(0, 3) = 0.0; H(0, 4) = 1.0; H(0, 5) = 0.0;
  H(1, 0) = 0.0; H(1, 1) = 1.0; H(1, 2) = 0.0; H(1, 3) = 0.0; H(1, 4) = 0.0; H(1, 5) = 1.0;
  H(2, 0) = 0.0; H(2, 1) = 0.0; H(2, 2) = 1.0; H(2, 3) = 0.0; H(2, 4) = 1.0; H(2, 5) = 0.0;
  H(3, 0) = 0.0; H(3, 1) = 0.0; H(3, 2) = 0.0; H(3, 3) = 1.0; H(3, 4) = 0.0; H(3, 5) = 1.0;
  H(4, 0) = -1.0; H(4, 1) = 0.0; H(4, 2) = 0.0; H(4, 3) = 0.0; H(4, 4) = 1.0; H(4, 5) = 0.0;
  H(5, 0) = 0.0; H(5, 1) = -1.0; H(5, 2) = 0.0; H(5, 3) = 0.0; H(5, 4) = 0.0; H(5, 5) = 1.0;
  H(6, 0) = 0.0; H(6, 1) = 0.0; H(6, 2) = -1.0; H(6, 3) = 0.0; H(6, 4) = 1.0; H(6, 5) = 0.0;
  H(7, 0) = 0.0; H(7, 1) = 0.0; H(7, 2) = 0.0; H(7, 3) = -1.0; H(7, 4) = 0.0; H(7, 5) = 1.0;
  parallelogramMatrix = H * (H.transpose() * H).inverse() * H.transpose();
}
///////////////////////////////////////////////////////////////////////////////
void ParallelogramConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block<3, 4>(0, idO_) = projection * weight_;
}
///////////////////////////////////////////////////////////////////////////////
void ParallelogramConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
  double coef2 = - weight_ / nIndices();
  for (int i = 0; i < nIndices(); ++i) {
    for (int j = 0; j < nIndices(); ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}

Matrix3X ParallelogramConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix34 input;
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Eigen::JacobiSVD<Matrix3X> jSVD;
    jSVD.compute(input, Eigen::ComputeFullU);
    Matrix33 basis = jSVD.matrixU();
    input = basis.transpose() * input;
    input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
    //Check that the coordinate system is in the good direction if not flip it
    bool flip = (input(0, 0) * input(1, 1) - input(0, 1) * input(1, 0)) < 0.0;
    if (flip)
      for (int j = 0; j < 4; ++j)
        input(1, j) = -input(1, j);
    //////
    MatrixT<8, 1> o;
    for (int j = 0; j < 4; ++j) {
      o(j * 2 + 0, 0) = input(0, j);
      o(j * 2 + 1, 0) = input(1, j);
    }
    //mutliply the 3 2d points stacked in o with deformation, corresponds to projection(o)
    MatrixT<8, 1> r = parallelogramMatrix * o;
    //reprojection into 3d
    Matrix34 output;
    for (int j = 0; j < 4; ++j) {
      output(0, j) = r(j * 2 + 0, 0);
      if (flip)
        output(1, j) = -r(j * 2 + 1, 0);
      else
        output(1, j) = r(j * 2 + 1, 0);
      output(2, j) = 0.0;
    }
    return basis * input;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
UniformLaplacianConstraint::UniformLaplacianConstraint(const std::vector<int> &idI,
                                                                      Scalar weight,
                                                                      const Matrix3X &positions,
                                                                      bool displacement_lap) :  Constraint(idI, weight) {
    this->displacement = displacement_lap;
    rest_.setZero();
    if (displacement_lap) {
        for (int i = 1; i < idI_.size(); ++i) rest_ += positions.col(idI[i]);
        rest_ /= double(idI_.size() - 1);
        rest_ -= positions.col(idI[0]);
    }
}

///////////////////////////////////////////////////////////////////////////////
void UniformLaplacianConstraint::project(const Matrix3X & positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.col(idO_) = weight_ * projection;
}

///////////////////////////////////////////////////////////////////////////////
void UniformLaplacianConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    idO_ = idO;
    triplets.push_back(Triplet(idO_, idI_[0], -weight_));
    double c = weight_ / (idI_.size() - 1);
    for (int i = 1; i < static_cast<int>(idI_.size()); ++i)
        triplets.push_back(Triplet(idO_, idI_[i], c));
    idO++;
}

Matrix3X UniformLaplacianConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    projection.resize(3, 1);
    Vector3 laplacian;
    laplacian.setZero();
    if (displacement) {
        for (int i = 1; i < idI_.size(); ++i) laplacian += positions.col(idI_[i]);
        laplacian /= double(idI_.size() - 1);
        laplacian -= positions.col(idI_[0]);
        laplacian.normalize();
        laplacian *= rest_.norm();
    }
    projection.col(0) = laplacian;

    return projection;
}

Vector3 UniformLaplacianConstraint::rest() const
{
    return rest_;
}

void UniformLaplacianConstraint::setRest(const Vector3 &weighted_rest)
{
    rest_ = weighted_rest;
}

std::size_t UniformLaplacianConstraint::nIndices() const
{
    return 1;
}

bool UniformLaplacianConstraint::getDisplacement() const
{
    return displacement;
}

void UniformLaplacianConstraint::setDisplacement(bool value)
{
    displacement = value;
}


///////////////////////////////////////////////////////////////////////////////
CotangentLaplacianConstraint::CotangentLaplacianConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions, bool displacement_lap) :
    Constraint(idI, weight)
{
    double totalArea = 0.0;
    double eps = 1e-6;
    this->displacement = displacement_lap;
    input.resize(3, idI_.size());
    cotWeights.setZero(idI_.size());
    for(unsigned int i = 0; i < idI_.size(); i++) input.col(i) = positions.col(idI[i]);
    Vector3 p0 = positions.col(idI[0]);
    for(unsigned int j = 1; j < idI_.size(); j++){
        Vector3 p1 = input.col(j);
        Vector3 p2 = j > 1         ? input.col(j - 1) : input.col(idI_.size() - 1);
        Vector3 p3 = j + 1 < idI_.size() ? input.col(j + 1) : input.col(1);
        Vector3 v1 = p0 - p2;
        Vector3 v2 = p1 - p2;
        Vector3 v3 = p0 - p3;
        Vector3 v4 = p1 - p3;
        double area = v1.cross(v2).norm();
        totalArea += area / 3.0;
        double cotan_alfa = v1.dot(v2) / area;
        double cotan_beta = v3.dot(v4) / v3.cross(v4).norm();

        double wj = cotan_alfa + cotan_beta;
        if(isnanf(wj))
            wj = 0.0;
        double cotan_max = cos(eps) / sin(eps);
        wj = clamp(wj, -cotan_max, cotan_max);
        cotWeights(j) = wj;
    }

    cotWeights /= cotWeights.sum();
    cotWeights(0) = -1;
    rest_ = input * cotWeights;

}
///////////////////////////////////////////////////////////////////////////////
void CotangentLaplacianConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);

    projections.col(idO_) = weight_ * projection;
}

///////////////////////////////////////////////////////////////////////////////
void CotangentLaplacianConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    idO_ = idO;

    for (int j = 0; j < static_cast<int>(idI_.size()); ++j)
        triplets.push_back(Triplet(idO_, idI_[j], weight_ * cotWeights(j)));

    idO++;
}

Matrix3X CotangentLaplacianConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, 1);

    Vector3 laplacian;
    laplacian.setZero();
    if (displacement) {
        for (int i = 0; i < idI_.size(); ++i) input.col(i) = positions.col(idI_[i]);
        laplacian = input * cotWeights;
        laplacian.normalize();
        laplacian *= rest_.norm();

    }
    projection.col(0) = laplacian;
    return projection;
}

std::size_t CotangentLaplacianConstraint::nIndices() const
{
    return 1;
}

bool CotangentLaplacianConstraint::getDisplacement() const
{
    return displacement;
}

void CotangentLaplacianConstraint::setDisplacement(bool value)
{
    displacement = value;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
AngleConstraint::AngleConstraint(const std::vector<int> &idI,
                                                Scalar weight,
                                                const Matrix3X &positions,
                                                Scalar minAngle,
                                                Scalar maxAngle) :
  Constraint(idI, weight) {
  setMinAngle(minAngle);
  setMaxAngle(maxAngle);
}
///////////////////////////////////////////////////////////////////////////////
void AngleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    projections.block<3, 2>(0, idO_) = weight_ * projection;
}
///////////////////////////////////////////////////////////////////////////////
void AngleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  triplets.push_back(Triplet(idO_, idI_[0], -weight_));
  triplets.push_back(Triplet(idO_, idI_[1], weight_));
  triplets.push_back(Triplet(idO_ + 1, idI_[0], -weight_));
  triplets.push_back(Triplet(idO_ + 1, idI_[2], weight_));
  idO += 2;
}

Matrix3X AngleConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());

    assert(minAngleCos_ >= maxAngleCos_);

    Vector3 v1 = positions.col(idI_[1]) - positions.col(idI_[0]),
            v2 = positions.col(idI_[2]) - positions.col(idI_[0]);

    projection.col(0) = v1;
    projection.col(1) = v2;

    double epsilon = 1e-14;
    double v1_sqrnorm = v1.squaredNorm(), v2_sqrnorm = v2.squaredNorm();
    double v1_norm = v1.norm(), v2_norm = v2.norm();

    Vector3 unit_v1 = v1, unit_v2 = v2;
    unit_v1.normalize();
    unit_v2.normalize();

    if (unit_v1.allFinite() && unit_v2.allFinite()) {
        // cosine value of the angle gamma between v1 and v2
        double cos_gamma = clamp(unit_v1.dot(unit_v2), -1.0, 1.0);

        // Proceed only when the current angle lies outside the target range, and v1, v2 are not colinear
        if ( (1.0 - std::abs(cos_gamma) > epsilon) && (cos_gamma > minAngleCos_ || cos_gamma < maxAngleCos_) ) {
            double gamma = std::acos(cos_gamma);

            // Angle eta: the sum of displacement angles from v1 and v2
            double eta = cos_gamma > minAngleCos_ ? (minAngle_ - gamma) : (gamma - maxAngle_);
            eta = (std::max)(eta, 0.0);

            // Compute angle theta between v1 and its projection, and angle phi between v2 and its projection
            double theta = 0.5 * std::atan2(v2_sqrnorm * std::sin(2 * eta), v1_sqrnorm + v2_sqrnorm * std::cos(2 * eta));
            theta = (std::max)(0.0, (std::min)(eta, theta));
            double phi = eta - theta;

            // Compute unit vectors that are coplanar with v1, v2, and orthogonal to one of them.
            // They form orthogonal frames with v1 and v2 respectively, within which we compute the projection using the above angles
            Vector3 unit_v3 = unit_v2 - unit_v1 * cos_gamma, unit_v4 = unit_v1 - unit_v2 * cos_gamma;
            unit_v3.normalize();
            unit_v4.normalize();

            // Determine if v1, v2 should move away from each other or towards each other
            if (cos_gamma > minAngleCos_) {
                unit_v3 *= -1.0;
                unit_v4 *= -1.0;
            }

            projection.col(0) = (unit_v1 * std::cos(theta) + unit_v3 * std::sin(theta)) * (v1_norm * std::cos(theta));
            projection.col(1) = (unit_v2 * std::cos(phi) + unit_v4 * std::sin(phi)) * (v2_norm * std::cos(phi));
        }
    }

    return projection;
}
///////////////////////////////////////////////////////////////////////////////
void AngleConstraint::setMinAngle(Scalar minAngle) {
    // Ensure the angle limits are between 0 and PI
    // Use parentheses to avoid conflicts with the min/max macros from windows.h
    minAngle_ = (std::max)(minAngle, 0.0);
    minAngleCos_ = clamp(std::cos(minAngle_), -1.0, 1.0);
}
///////////////////////////////////////////////////////////////////////////////
void AngleConstraint::setMaxAngle(Scalar maxAngle) {
    maxAngle_ = (std::min)(maxAngle, M_PI);
    maxAngleCos_ = clamp(std::cos(maxAngle_), -1.0, 1.0);
}

Scalar AngleConstraint::minAngle() const
{
    return minAngle_;
}

Scalar AngleConstraint::maxAngle() const
{
    return maxAngle_;
}

///////////////////////////////////////////////////////////////////////////////
OrientationConstraint::OrientationConstraint(const std::vector<int> &idI,
                                                        Scalar weight,
                                                        const Matrix3X &positions,
                                                        Vector3 normal /*= Vector3(0.0,0.0,1.0)*/):
  Constraint(idI,weight)
{
    assert(idI.size() >= 1);
    setOrientation(normal);
    //Allocate memory for intermediate storage during projection
    input = Matrix3X::Zero(3, idI.size());
}


///////////////////////////////////////////////////////////////////////////////
void OrientationConstraint::setOrientation(const Vector3 &normal){
    normal_ = normal;
    normal_.normalize();
}

///////////////////////////////////////////////////////////////////////////////
void OrientationConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
    Matrix3X projection = getProjection(positions);
    for(unsigned int i = 0; i < nIndices(); i++)
        projections.col(idO_ + i) = weight_ * projection.col(i);
}


///////////////////////////////////////////////////////////////////////////////
void OrientationConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }
}

Matrix3X OrientationConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());

    //Copy the constrained positions to input
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    //Compute and subtract mean
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    //Project each position onto the plane through zero defined by normal_
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
        projection.col(i) = (input.col(i) - normal_ * (normal_.dot(input.col(i))));

    return projection;
}

///////////////////////////////////////////////////////////////////////////////
RegressionPlaneOrientationConstraint::RegressionPlaneOrientationConstraint(const std::vector<int> &idI,
                                                                                          Scalar weight, const Matrix3X &positions, Vector3 normal) :
                                                                                          Constraint(idI, weight)
{
    assert(idI.size() >= 3);
    setOrientation(normal);
    //Allocate memory for intermediate storage during projection
    input = Matrix3X::Zero(3, idI.size());
}

///////////////////////////////////////////////////////////////////////////////
void RegressionPlaneOrientationConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    for(unsigned int i = 0; i < nIndices(); i++)
        projections.col(idO_ + i) = weight_ * projection.col(i);
}

///////////////////////////////////////////////////////////////////////////////
void RegressionPlaneOrientationConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }
}

Matrix3X RegressionPlaneOrientationConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());

    //Copy the constrained positions to input
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    //Compute and subtract mean
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Matrix33 U = input.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    double angle = std::acos(U.col(2).dot(normal_) / U.col(2).norm());
    Vector3 rotAxis = U.col(2).cross(normal_);
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

    for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
        projection.col(i) = (rotMatrix * input.col(i));

    return projection;
}

///////////////////////////////////////////////////////////////////////////////
void RegressionPlaneOrientationConstraint::setOrientation(const Vector3 &normal)
{
    normal_ = normal;
    normal_.normalize();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
PolylineLengthConstraint::PolylineLengthConstraint(const std::vector<int> &idI,
                                                                  Scalar weight,
                                                                  const Matrix3X &positions,
                                                                  const Scalar rangeMin,
                                                                  const Scalar rangeMax) : Constraint(idI, weight)
{
    rangeMin_ = rangeMin;
    rangeMax_ = rangeMax;
    assert(idI.size() >= 2);
    input = Matrix3X::Zero(3, idI.size());
}

///////////////////////////////////////////////////////////////////////////////
void PolylineLengthConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    for(unsigned int i = 0; i < nIndices(); i++)
        projections.col(idO_ + i) = weight_ * projection.col(i);
}

///////////////////////////////////////////////////////////////////////////////
void PolylineLengthConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }
}

Matrix3X PolylineLengthConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());

    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    //Compute and subtract mean
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    Scalar l = 0;
    std::vector<Vector3> edges;
    std::vector<Vector3> newPos;
    for(unsigned int i = 1; i < idI_.size(); i++){
        edges.push_back(input.col(i) - input.col(i - 1));
        l += edges[i - 1].norm();
    }

    double proportion = 1;
    if( l > rangeMax_)
        proportion = rangeMax_ / l;
    else if (l < rangeMin_)
        proportion = rangeMin_ / l;

    //std::cout << "Length: " << l << ", Range min: " << rangeMin_ << ", Range max: " << rangeMax_ << ", Proportion: " << proportion << std::endl << std::flush;
    newPos.push_back(input.col(0));
    Vector3 center = newPos[0];
    for(unsigned int i = 1; i < idI_.size(); i++){
        newPos.push_back(newPos[i - 1] + edges[i - 1] * proportion);
        center += newPos[i];
    }


    center /= idI_.size();

    for(unsigned int i = 0; i < idI_.size(); i++)
        projection.col(i) = (newPos[i] - center);

    return projection;
}

///////////////////////////////////////////////////////////////////////////////
Scalar PolylineLengthConstraint::getRangeMin() const
{
    return rangeMin_;
}

///////////////////////////////////////////////////////////////////////////////
void PolylineLengthConstraint::setRangeMin(const Scalar &value)
{
    rangeMin_ = value;
}

///////////////////////////////////////////////////////////////////////////////
Scalar PolylineLengthConstraint::getRangeMax() const
{
    return rangeMax_;
}

///////////////////////////////////////////////////////////////////////////////
void PolylineLengthConstraint::setRangeMax(const Scalar &value)
{
    rangeMax_ = value;
}

///////////////////////////////////////////////////////////////////////////////
RectangleFitConstraint::RectangleFitConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions) :
    Constraint(idI, weight)
{
    assert(idI.size() >= 4);
    input = Matrix3X::Zero(3, idI.size());
}

bool areEqual(double a, double b){
    bool result = false;
    if((a + 1e-10 > b) && (a - 1e-10 < b))
        result = true;

    return result;
}

bool isALessThanB(std::pair<std::pair<double, double>, unsigned int> a,
                                 std::pair<std::pair<double, double>, unsigned int> b){

    if(a.first.first < b.first.first)
        return true;
    else if(areEqual(a.first.first, b.first.first) && a.first.second < b.first.second)
        return true;
    return false;

}

bool isLeft(Vector3 p, Vector3 p1, Vector3 p2){
    return (p(0) - p1(0)) * (p2(1) - p1(1)) - (p(1) - p1(1)) * (p2(0) - p1(0)) < 0;
}

Vector3 pointLineProjection(Vector3 a, Vector3 b, Vector3 c){
    Vector3 abVector = (b - a);
    return a + ((c - a).dot(abVector) / abVector.dot(abVector)) * abVector;
}

/**
 * @brief pointLineDistance distance between the point c and the line passing through a and b
 * @param a first point through which the line passes
 * @param b second point through which the line passes
 * @param c the point whose distance from the line we are interested in
 * @return the distance
 */
double pointLineDistance(Vector3 a, Vector3 b, Vector3 c){
    return (c - a).cross(c - b).norm() / (b - a).norm();
}

bool isPointInSegment(Vector3 a, Vector3 b, Vector3 p)
{
    double l = (b - a).norm();
    double l1 = (p - a).norm();
    double l2 = (b - p).norm();
    double w1 = l1 / l;
    double w2 = l2 / l;

    return w1 + w2 <= 1.0 + 1e-5;
}

Vector3 computeLineIntersection(Vector3 v1, Vector3 p1, Vector3 v2, Vector3 p2){
    double x;
    double a, b, c, d;
    bool aUndefined = false, bUndefined = false;

    if(areEqual(v1(0), 0)){
        x = p1(0);
        aUndefined = true;
    }else{
        a = v1(1) / v1(0);
        c = (-a) * p1(0) + p1(1);
    }
    if(areEqual(v2(0), 0)){
        x = p2(0);
        bUndefined = true;
    }else{
        b = v2(1) / v2(0);
        d = (-b) * p2(0) + p2(1);
    }

    if(!(aUndefined || bUndefined))
        x = (d - c) / (a - b);

    double y;

    if(aUndefined)
        y = b * x + d;
    else
        y = a * x + c;

    Vector3 intersection = {x, y, 0};
    return intersection;
}


/**
 * @brief extractConvexHull this function is based on the Graham's Algorithm.
 * It works with 3d points placed on the 2d plane (z must always be 0)
 * @param points the points from which the CH should be extracted
 * @return the list of points composing the CH
 */
std::vector<Vector3> extractConvexHull(MatrixX3 points){


    unsigned int lowestLeftmostPosition = 0;
    double smallestY = DBL_MAX, smallestX = DBL_MAX;
    std::vector<Vector3> convexHull;

    std::vector<std::pair<std::pair<double, double>, unsigned int> > segmentsAngleAndLength;
    for(unsigned int i = 0; i < static_cast<unsigned int>(points.rows()); i++)
        if(points(i,1) < smallestY){
            lowestLeftmostPosition = i;
            smallestY = points(i,1);
        }else if(areEqual(points(i,1), smallestY) && points(i,0) < smallestX){
            lowestLeftmostPosition = i;
            smallestX = points(i,0);
        }

    Vector3 xAxis = {1, 0, 0};
    for(unsigned int i = 0; i < static_cast<unsigned int>(points.rows()); i++){

        if(i == lowestLeftmostPosition)
            continue;

        Vector3 pi = points.row(i);
        Vector3 tmp = points.row(lowestLeftmostPosition);
        Vector3 v = pi - tmp;
        v.normalize();
        double angle = acos(xAxis.dot(v) / (xAxis.norm() * v.norm()));

        std::pair<double, double> angleAndLength = std::make_pair(angle, v.norm());
        std::pair<std::pair<double, double>, unsigned int> segmentAngleAndLength = make_pair(angleAndLength, i);
        segmentsAngleAndLength.push_back(segmentAngleAndLength);
    }

    std::sort(segmentsAngleAndLength.begin(), segmentsAngleAndLength.end(), isALessThanB);

    convexHull.push_back(points.row(lowestLeftmostPosition));
    convexHull.push_back(points.row(segmentsAngleAndLength[0].second));

    unsigned int i = 1;
    while(i < segmentsAngleAndLength.size()){

        unsigned int actualPointPosition = segmentsAngleAndLength[i].second;
        Vector3 p = points.row(actualPointPosition);
        Vector3 p1 = convexHull.at(convexHull.size() - 2);
        Vector3 p2 = convexHull.back();
        if(isLeft(p, p1, p2)){
            convexHull.push_back(p);
            i++;
        }else
            convexHull.pop_back();


    }

    convexHull.push_back(convexHull.front());

    return convexHull;

}

/**
 * @brief getMBR this function is based on the Rotating Calipers Algorithm.
 * It works with 3d points placed on the 2d plane (z must always be 0)
 * @param points the points from which the MBR should be extracted
 * @return
 */
MatrixX3 getMBR(MatrixX3 points){

    Vector3 xAxis = {1.0, 0.0, 0.0};
    double minArea = DBL_MAX;
    MatrixX3 MBR;
    std::vector<Vector3> ch = extractConvexHull(points);

    std::vector<std::pair<unsigned int, unsigned int> > chEdges;
    chEdges.push_back(std::make_pair(ch.size() - 1, 0));
    for(unsigned int i = 1; i < ch.size(); i++)
        chEdges.push_back(std::make_pair(i - 1, i));

    for(unsigned int i = 0; i < chEdges.size(); i++)
    {
        Matrix34 rectangle;
        Vector3 edgeU0 = (ch[chEdges[i].first] - ch[chEdges[i].second]).normalized();
        Vector3 edgeU1 = {edgeU0(1), -edgeU0(0), 0};
        double angle = acos(xAxis.dot(edgeU0));
        Matrix33 rotationMatrix;
        rotationMatrix << cos(angle), -sin(angle), 0,
                          sin(angle),  cos(angle), 0,
                                   0,           0, 0;
        double minX = DBL_MAX, maxX = -DBL_MAX, minY = DBL_MAX, maxY = -DBL_MAX;
        for(unsigned int j = 0; j < ch.size(); j++){
            Vector3 convertedPoint = rotationMatrix * (ch[j] - ch[chEdges[i].first]);
            if(convertedPoint(0) < minX)
                minX = convertedPoint(0);
            if(convertedPoint(0) > maxX)
                maxX = convertedPoint(0);
            if(convertedPoint(1) < minY)
                minY = convertedPoint(1);
            if(convertedPoint(1) > maxY)
                maxY = convertedPoint(1);
        }

        rectangle << minX, maxX, maxX, minX,
                     minY, minY, maxY, maxY,
                        0,    0,    0,    0;
        double area = (rectangle.col(1) - rectangle.col(0)).cross(rectangle.col(2) - rectangle.col(0)).norm();
        rectangle = (rotationMatrix.transpose() * rectangle).colwise() + ch[chEdges[i].first];
        if(area < minArea){
            minArea = area;
            MBR = rectangle.transpose();
        }


    }

    return MBR;

}


///////////////////////////////////////////////////////////////////////////////
void RectangleFitConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);

    for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
        projections.col(idO_ + i) = projection.col(i) * weight_;

}

///////////////////////////////////////////////////////////////////////////////
void RectangleFitConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }
}

Matrix3X RectangleFitConstraint::getProjection(const Matrix3X &positions) const
{

    Vector3 normal = {0, 0, 1};

    //Copy the constrained positions to input
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
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


    Matrix3X planarInput;
    planarInput.resize(3, nIndices());
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i){
        planarInput.col(i) = (rotMatrix * input.col(i));
        planarInput(2, i) = 0;
    }

    MatrixX3 MBR = getMBR(planarInput.transpose());

    std::vector<unsigned int> clusters[4];
    for(unsigned int i = 0; i < idI_.size(); i++){
        double bestDistance = DBL_MAX;
        unsigned int associatedCluster = 4;
        for(unsigned int j = 0; j < 4; j++){
            double dist = pointLineDistance(MBR.row(j), MBR.row((j + 1) % 4), planarInput.col(i));

            if( dist < bestDistance ){
                bestDistance = dist;
                associatedCluster = j;
            }
        }

        clusters[associatedCluster].push_back(i);
    }

    for(unsigned int i = 0; i < 4; i++)
    {
        Vector3 averagePoint = {0, 0, 0};
        for(unsigned int j = 0; j < clusters[i].size(); j++)
            averagePoint += planarInput.col(clusters[i][j]);
        averagePoint /= clusters[i].size();
        Vector3 projectedAveragePoint1 = pointLineProjection(MBR.row((i + 1) % 4), MBR.row((i + 2) % 4), averagePoint);
        Vector3 sideVector = MBR.row((i + 1) % 4) - MBR.row(i);
        Vector3 projectedAveragePoint2 = projectedAveragePoint1 - sideVector;
        MBR.row((i + 1) % 4) = projectedAveragePoint1;
        MBR.row(i) = projectedAveragePoint2;
    }

    for(unsigned int i = 0; i < 4; i++)
    {
        Vector3 a = MBR.row(i), b = MBR.row((i + 1) % 4);
        for(unsigned int j = 0; j < clusters[i].size(); j++){

            Vector3 projected = pointLineProjection(a, b, planarInput.col(clusters[i][j]));

            if(!isPointInSegment(a, b, projected)){
                double dist1 = (projected - a).norm();
                if( dist1 < (projected - b).norm())
                    projected = a;
                else
                    projected = b;
            }

            planarInput.col(clusters[i][j]) = projected;

        }
    }

    return rotMatrix.transpose() * planarInput;
}

///////////////////////////////////////////////////////////////////////////////
EquilateralConstraint::EquilateralConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions) :
    Constraint(idI, weight){

    assert(idI.size() == 3);

    input.resize(3, 3);
}

///////////////////////////////////////////////////////////////////////////////
void EquilateralConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    for(unsigned int i = 0; i < nIndices(); i++)
        projections.col(idO_ + i) = projection.col(i) * weight_;
}

///////////////////////////////////////////////////////////////////////////////
void EquilateralConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j){
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        }
        idO++;
    }
}

Matrix3X EquilateralConstraint::getProjection(const Matrix3X &positions) const
{

    Matrix3X projection;
    projection.resize(3, nIndices());

    for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
    //Compute and subtract mean
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;

    Matrix33 sides;
    int pos = -1;

    for(unsigned int i = 0; i < 3; i++)
        sides.col(i) = input.col((i + 1) % 3) - input.col(i);

    double sideLength = sides.colwise().norm().minCoeff(&pos);

    projection.col(pos) = input.col(pos);
    projection.col((pos + 1) % 3) = input.col(pos) + sides.col(pos).normalized() * sideLength;
    projection.col((pos + 2) % 3) = input.col((pos + 1) % 3) + sides.col((pos + 1) % 3).normalized() * sideLength;


    return projection;
}

SimilarLengthConstraint::SimilarLengthConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions, Scalar minProportion, Scalar maxProportion):
    Constraint(idI, weight),
    minProportion_(minProportion),
    maxProportion_(maxProportion)
{
    linePointsCardinality1 = idI.size() / 2;
    linePointsCardinality2 = idI.size() / 2;
    input.resize(3, idI.size());
}

SimilarLengthConstraint::~SimilarLengthConstraint(){}

void SimilarLengthConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, linePointsCardinality1+linePointsCardinality2) = projection * weight_;
}

void SimilarLengthConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (unsigned int i = 0; i < nIndices(); ++i) {
        for (unsigned int j = 0; j < nIndices(); ++j){
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        }
        idO++;
    }
}

Matrix3X SimilarLengthConstraint::getProjection(const Matrix3X &positions) const
{

//    std::cout << "Nuovo giro nuova corsa" << std::endl;
    Matrix3X projection;
    Matrix3X newPos;
    Matrix3X edges;
    newPos.resize(3, linePointsCardinality2);
    edges.resize(3, linePointsCardinality2 - 1);
    for (int i = 0; i < linePointsCardinality1 + linePointsCardinality2; ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;
    projection.resize(3, linePointsCardinality1 + linePointsCardinality2);
    Scalar l1 = 0.0, l2 = 0.0;

    for(unsigned int i = 1; i < linePointsCardinality1; i++)
        l1 += (input.col(i) - input.col(i - 1)).norm();

    for(unsigned int i = 1; i < linePointsCardinality2; i++)
        edges.col(i - 1) = (input.col(linePointsCardinality1 + i) - input.col(linePointsCardinality1 + i - 1));

    l2 = edges.norm();

    double proportion = 1;
    if(l2 > l1 * maxProportion_)
        proportion = l1 * maxProportion_ / l2;
    else if (l2 < l1 * minProportion_)
        proportion = l1 *  minProportion_ / l2;


    newPos.col(0) = (input.col(linePointsCardinality1));
    for(unsigned int i = 1; i < linePointsCardinality2; i++)
        newPos.col(i) = newPos.col(i - 1) + edges.col(i - 1) * proportion;

    projection.block(0, 0, 3, linePointsCardinality1) = input.block(0,0, 3, linePointsCardinality1);
    projection.block(0, linePointsCardinality1, 3, linePointsCardinality2) = newPos;/*
    std::cout << "First polyline:" << std::endl << input.block(0,0, 3, linePointsCardinality1) << std::endl << std::endl;
    std::cout << "Second polyline:" << std::endl << input.block(0,linePointsCardinality1, 3, linePointsCardinality2) << std::endl << std::endl;
    std::cout << "New positions second polyline" << std::endl << newPos << std::endl << std::flush << std::endl;
    std::cout << "Final projection" << std::endl << projection << std::endl << std::endl << std::flush;

    int a = 0;*/

    return projection;
}

unsigned int SimilarLengthConstraint::getLinePointsCardinality1() const
{
    return linePointsCardinality1;
}

void SimilarLengthConstraint::setLinePointsCardinality1(unsigned int value)
{
    linePointsCardinality1 = value;
}

unsigned int SimilarLengthConstraint::getLinePointsCardinality2() const
{
    return linePointsCardinality2;
}

void SimilarLengthConstraint::setLinePointsCardinality2(unsigned int value)
{
    linePointsCardinality2 = value;
}

double SimilarLengthConstraint::getMinProportion() const
{
    return minProportion_;
}

void SimilarLengthConstraint::setMinProportion(double value)
{
    minProportion_ = value;
}

std::size_t SimilarLengthConstraint::nIndices() const
{
    return idI_.size();
}

double SimilarLengthConstraint::getMaxProportion() const
{
    return maxProportion_;
}

void SimilarLengthConstraint::setMaxProportion(double value)
{
    maxProportion_ = value;
}

NonUniformScalingConstraint::NonUniformScalingConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions, Scalar minProportion, Scalar maxProportion, int l1StartIndex, int l1EndIndex, int l2StartIndex, int l2EndIndex, bool l1directed, bool l2directed, Vector3 direction1, Vector3 direction2):
    Constraint(idI, weight),
    minProportion_(minProportion),
    maxProportion_(maxProportion),
    l1StartIndex_(l1StartIndex),
    l1EndIndex_(l1EndIndex),
    l2StartIndex_(l2StartIndex),
    l2EndIndex_(l2EndIndex),
    l1directed_(l1directed),
    l2directed_(l2directed),
    l1direction_(direction1),
    l2direction(direction2)
{
    input.resize(3, idI.size());
}

NonUniformScalingConstraint::~NonUniformScalingConstraint()
{

}

void NonUniformScalingConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    projections.block(0, idO_, 3, input.cols()) = projection * weight_;
}

void NonUniformScalingConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    //Store this constraints position in the global linear system
    idO_ = idO;
    //Precompute coefficients for mean-centering
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = -weight_ / nIndices();
    //Add triplets to the sparse global linear system.
    for (unsigned int i = 0; i < nIndices(); ++i) {
        for (unsigned int j = 0; j < nIndices(); ++j){
            //Add the coefficent for mean-centering to the sparse linear system at column id0 and row idI_[j].
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        }
        idO++;
    }
}

Matrix3X NonUniformScalingConstraint::getProjection(const Matrix3X &positions) const
{
    Matrix3X projection;
    Vector3 origin = {0,0,0};
    projection.resize(3, input.cols());

    for (int i = 0; i < input.cols(); ++i) input.col(i) = positions.col(idI_[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;

    Scalar l1, l2;
    Vector3 scalingDirection;
    if(l1directed_)
    {
        Vector3 l1p1 = pointLineProjection(origin, l1direction_, positions.col(l1StartIndex_));
        Vector3 l1p2 = pointLineProjection(origin, l1direction_, positions.col(l1EndIndex_));
        l1 = (l1p2 - l1p1).norm();
        scalingDirection = l1direction_;
    } else
    {
        scalingDirection = positions.col(l1EndIndex_) - positions.col(l1StartIndex_);
        l1 = scalingDirection.norm();
        scalingDirection.normalize();
    }

    if(l2directed_)
    {
        Vector3 l2p1 = pointLineProjection(origin, l2direction, positions.col(l2StartIndex_));
        Vector3 l2p2 = pointLineProjection(origin, l2direction, positions.col(l2EndIndex_));
        l2 = (l2p2 - l2p1).norm();
    } else {
        l2 = (positions.col(l2EndIndex_) - positions.col(l2StartIndex_)).norm();
    }

    double proportion = 1;
    if(l1 > l2 * maxProportion_)
        proportion = l2 * maxProportion_ / l1;
    else if (l1 < l2 * minProportion_)
        proportion = l2 *  minProportion_ / l1;
    for(unsigned int i = 0; i < input.cols(); i++){
        Vector3 projected = pointLineProjection(origin, scalingDirection, input.col(i));
        Vector3 inverseProjection = input.col(i) - projected;
        projected *= proportion;
        projection.col(i) = projected + inverseProjection;
    }

    return projection;
}


double NonUniformScalingConstraint::getMinProportion() const
{
    return minProportion_;
}

void NonUniformScalingConstraint::setMinProportion(double minProportion)
{
    minProportion_ = minProportion;
}

double NonUniformScalingConstraint::getMaxProportion() const
{
    return maxProportion_;
}

void NonUniformScalingConstraint::setMaxProportion(double maxProportion)
{
    maxProportion_ = maxProportion;
}

int NonUniformScalingConstraint::getL1StartIndex() const
{
    return l1StartIndex_;
}

void NonUniformScalingConstraint::setL1StartIndex(int l1StartIndex)
{
    l1StartIndex_ = l1StartIndex;
}

int NonUniformScalingConstraint::getL1EndIndex() const
{
    return l1EndIndex_;
}

void NonUniformScalingConstraint::setL1EndIndex(int l1EndIndex)
{
    l1EndIndex_ = l1EndIndex;
}

int NonUniformScalingConstraint::getL2StartIndex() const
{
    return l2StartIndex_;
}

void NonUniformScalingConstraint::setL2StartIndex(int l2StartIndex)
{
    l2StartIndex_ = l2StartIndex;
}

int NonUniformScalingConstraint::getL2EndIndex() const
{
    return l2EndIndex_;
}

void NonUniformScalingConstraint::setL2EndIndex(int l2EndIndex)
{
    l2EndIndex_ = l2EndIndex;
}

Vector3 NonUniformScalingConstraint::getL1direction() const
{
    return l1direction_;
}

void NonUniformScalingConstraint::setL1direction(const Vector3 &l1direction)
{
    l1direction_ = l1direction;
}

Vector3 NonUniformScalingConstraint::getL2direction() const
{
    return l2direction;
}

void NonUniformScalingConstraint::setL2direction(const Vector3 &value)
{
    l2direction = value;
}

bool NonUniformScalingConstraint::getL1directed() const
{
    return l1directed_;
}

void NonUniformScalingConstraint::setL1directed(bool l1directed)
{
    l1directed_ = l1directed;
}

bool NonUniformScalingConstraint::getL2directed() const
{
    return l2directed_;
}

void NonUniformScalingConstraint::setL2directed(bool l2directed)
{
    l2directed_ = l2directed;
}


CoaxialityConstraint::CoaxialityConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions, Scalar minValue, Scalar maxValue, unsigned int firstSetSize, unsigned int secondSetSize) :
    Constraint(idI, weight),
    minValue_(minValue),
    maxValue_(maxValue),
    firstSetSize_(firstSetSize),
    secondSetSize_(secondSetSize)
{

}

CoaxialityConstraint::~CoaxialityConstraint()
{

}

void CoaxialityConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
{
    Matrix3X projection = getProjection(positions);
    for(unsigned int i = 0; i < nIndices(); i++)
        projections.col(idO_ + i) = weight_ * projection.col(i);
}

void CoaxialityConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
{
    idO_ = idO;
    double coef1 = (1.0 - 1.0 / nIndices()) * weight_;
    double coef2 = - weight_ / nIndices();
    for (int i = 0; i < nIndices(); ++i) {
        for (int j = 0; j < nIndices(); ++j)
            triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
        idO++;
    }


}


std::pair<unsigned int, std::vector<Vector3>> CoaxialityConstraint::sphere_line_intersection (Vector3 p1, Vector3 p2, Vector3 p3, double r, bool inSegment) const
{

    std::vector<Vector3> intersections;
    unsigned int intersections_number = 0;
    double a, b, c, mu, i, epsilon = 1e-5;

    a =  pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2) + pow(p2.z() - p1.z(), 2);
    b =  2 * ((p2.x() - p1.x()) * (p1.x() - p3.x()) +
              (p2.y() - p1.y()) * (p1.y() - p3.y()) +
              (p2.z() - p1.z()) * (p1.z() - p3.z()));
    c = pow(p3.x(), 2) + pow(p3.y(), 2) + pow(p3.z(), 2) + pow(p1.x(), 2) +
        pow(p1.y(), 2) + pow(p1.z(), 2) - 2 * (p3.x() * p1.x() + p3.y() * p1.y() + p3.z() * p1.z()) - pow(r, 2);
    i = pow(b, 2) - 4 * a * c;
    if ( i < 0.0 - epsilon)
        return(std::make_pair(intersections_number, intersections));

    mu = (-b + sqrt(i)) / (2 * a) ;
    Vector3 intersection = { p1.x() + mu * (p2.x() - p1.x()),
                             p1.y() + mu * (p2.y() - p1.y()),
                             p1.z() + mu * (p2.z() - p1.z())};
    if(!inSegment || isPointInSegment(p1, p2, intersection))
    {
        intersections_number++;
        intersections.push_back(intersection);
    }

    if ( i > 0.0 + epsilon)
    {
        mu = (-b - sqrt(i)) / (2 * a);

        Vector3 intersection2 = {p1.x() + mu * (p2.x() - p1.x()),
                                 p1.y() + mu * (p2.y() - p1.y()),
                                 p1.z() + mu * (p2.z() - p1.z())};
        if(!inSegment || isPointInSegment(p1, p2, intersection2))
        {
            intersections_number++;
            intersections.push_back(intersection2);
        }
    }
    return std::make_pair(intersections_number, intersections);

}

Vector3 imatiToEigenPoint(IMATI_STL::Point v){
    Vector3 p = {v.x, v.y, v.z};
    return p;
}

Matrix3X CoaxialityConstraint::extractSkeleton(std::vector<unsigned int> seedLoop) const
{
    int edge_flag = EDGE_FLAGGED;
    int triangle_inserted_flag = TRIANGLE_INSERTED;
    int triangle_used_flag = TRIANGLE_FLAGGED;
    Matrix3X skeleton;
    std::vector<Vector3> nodes;
    std::vector<IMATI_STL::Triangle*> usedTriangles;

    std::vector<IMATI_STL::Triangle*> loopTriangles;
    std::vector<Vector3> intersectionPoints;

    IMATI_STL::Vertex* v1 = mesh->getPoint(seedLoop[0]);
    for(unsigned int i = 1; i < seedLoop.size(); i++){
        IMATI_STL::Vertex* v2 = mesh->getPoint(seedLoop[i]);
        loopTriangles.push_back(v1->getEdge(v2)->leftTriangle(v1));
        Vector3 p = {v1->x,  v1->y, v1->z};
        intersectionPoints.push_back(p);
        v1 = v2;
    }

    Vector3 node;
    do{
        node.setZero();
        for(unsigned int i = 0; i < intersectionPoints.size(); i++)
            node += intersectionPoints[i];
        node /= intersectionPoints.size();
        nodes.push_back(node);
        double radius = 0.0;
        for(unsigned int i = 0; i < intersectionPoints.size(); i++)
        {
            double dist = (node - intersectionPoints[i]).norm();
            if(radius < dist)
                radius = dist;
        }

        radius += radius / 2;

        std::queue<IMATI_STL::Triangle*> Q;

        loopTriangles[0]->info = &triangle_inserted_flag;
        usedTriangles.push_back(loopTriangles[0]);
        for(unsigned int i = 0; i < loopTriangles.size(); i++)
        {
            Q.push(loopTriangles[i]);
            loopTriangles[i]->info = &triangle_inserted_flag;
        }

        loopTriangles.clear();
        intersectionPoints.clear();
        while(Q.size() > 0)
        {
            bool intersected = false;
            IMATI_STL::Triangle* t = Q.front();
            Q.pop();
            if(t->info != nullptr && *static_cast<int*>(t->info) == triangle_used_flag)
                continue;
            t->info = &triangle_used_flag;
            usedTriangles.push_back(t);
            IMATI_STL::Edge* e = t->e1;
            for(unsigned int i = 0; i < 3; i++){
                if(e->info == nullptr || *static_cast<int*>(e->info) != edge_flag) {
                    std::pair<unsigned int, std::vector<Vector3>> intersections = sphere_line_intersection(imatiToEigenPoint(e->v1), imatiToEigenPoint(e->v2), node, radius, true);
                    if(intersections.first > 0)
                    {
                        intersectionPoints.push_back(intersections.second[0]);
                        if(intersections.first == 2)
                            intersectionPoints.push_back(intersections.second[1]);
                        intersected = true;
                    }
                    else {
                        IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
                        if(!(t_->info != nullptr && (*static_cast<int*>(t_->info) == triangle_used_flag || *static_cast<int*>(t_->info) == triangle_inserted_flag)))
                        {
                            t_->info = &triangle_inserted_flag;
                            Q.push(t_);
                        }
                    }
                }
                e = t->nextEdge(e);
            }
            if(intersected)
                loopTriangles.push_back(t); //No need to test presence: triangles are used only once
        }
    } while(loopTriangles.size() != 0);

    skeleton.resize(3, nodes.size());

    for(unsigned int i = 0; i < nodes.size(); i++)
        skeleton.col(i) = nodes[i];

    for(std::vector<IMATI_STL::Triangle*>::iterator it = usedTriangles.begin(); it != usedTriangles.end(); it++){
        IMATI_STL::Triangle* t = *it;
        t->info = nullptr;
        t->e1->info = nullptr;
        t->e2->info = nullptr;
        t->e3->info = nullptr;
    }

    return skeleton;
}

Matrix3X CoaxialityConstraint::getProjection(const Matrix3X &positions) const
{
    int edge_flag = EDGE_FLAGGED;

    Matrix3X projection, firstSet, secondSet;
    std::vector<unsigned int> firstSetIndices, secondSetIndices;
    input.resize(3, nIndices());
    projection.resize(3, nIndices());
    firstSet.resize(3, firstSetSize_);
    secondSet.resize(3, secondSetSize_);

    //Copy the constrained positions to input
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
        input.col(i) = positions.col(idI_[i]);

    Vector3 mean = input.rowwise().mean();
    input.colwise() -= mean;
    firstSet = input.block(0, 0, 3, firstSetSize_);
    secondSet = input.block(0, firstSetSize_, 3, secondSetSize_);

    Vector3 firstSetMean =  firstSet.rowwise().mean();
    firstSet.colwise() -= firstSetMean;
    Vector3 secondSetMean = secondSet.rowwise().mean();
    secondSet.colwise();

    for(unsigned int i = 0; i < firstSetOutlines.size(); i++){
        IMATI_STL::Vertex* v1 = mesh->getPoint(firstSetOutlines[i][0]);
        for(unsigned int j = 1; j < firstSetOutlines[i].size(); j++)
        {
            IMATI_STL::Vertex* v2 = mesh->getPoint(firstSetOutlines[i][j]);
            v1->getEdge(v2)->info = &edge_flag;
            v1=v2;
        }
    }


    for(unsigned int i = 0; i < secondSetOutlines.size(); i++){
        IMATI_STL::Vertex* v1 = mesh->getPoint(secondSetOutlines[i][0]);
        for(unsigned int j = 1; j < secondSetOutlines[i].size(); j++)
        {
            IMATI_STL::Vertex* v2 = mesh->getPoint(secondSetOutlines[i][j]);
            v1->getEdge(v2)->info = &edge_flag;
            v1=v2;
        }
    }

    Matrix3X firstSetSkeleton = extractSkeleton(firstSetOutlines[firstSetSeedLoop]);
    Matrix3X secondSetSkeleton = extractSkeleton(secondSetOutlines[secondSetSeedLoop]);

    for(unsigned int i = 0; i < firstSetOutlines.size(); i++){
        IMATI_STL::Vertex* v1 = mesh->getPoint(firstSetOutlines[i][0]);
        for(unsigned int j = 1; j < firstSetOutlines[i].size(); j++)
        {
            IMATI_STL::Vertex* v2 = mesh->getPoint(firstSetOutlines[i][j]);
            v1->getEdge(v2)->info = nullptr;
            v1=v2;
        }
    }


    for(unsigned int i = 0; i < secondSetOutlines.size(); i++){
        IMATI_STL::Vertex* v1 = mesh->getPoint(secondSetOutlines[i][0]);
        for(unsigned int j = 1; j < secondSetOutlines[i].size(); j++)
        {
            IMATI_STL::Vertex* v2 = mesh->getPoint(secondSetOutlines[i][j]);
            v1->getEdge(v2)->info = nullptr;
            v1=v2;
        }
    }
    std::pair<Vector3, Vector3> axis, axis1, axis2;
    firstSetSkeleton.colwise() -= firstSetSkeleton.rowwise().mean();
    secondSetSkeleton.colwise() -= secondSetSkeleton.rowwise().mean();

    axis1.first = firstSetMean;
    axis2.first = secondSetMean;
    Matrix33 U = firstSetSkeleton.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    axis1.second = U.col(0);
    U = secondSetSkeleton.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    axis2.second = U.col(0);

    double dotProduct = axis1.second.dot(axis2.second);
    if(dotProduct < 0)
    {
        axis1.second *= -1;
        dotProduct = axis1.second.dot(axis2.second);
    }
    Scalar angle = acos(dotProduct);
    Vector3 rotAxis = axis1.second.cross(axis2.second).normalized();
    Matrix33 rotMatrix = createRotationMatrix(rotAxis, angle);
    Vector3 translationVector = pointLineProjection(axis2.first, axis2.first + axis2.second, axis1.first) - axis1.first;
    firstSet = rotMatrix * firstSet;
    firstSet.colwise() += firstSetMean + translationVector;

    projection.block(0, 0, 3, firstSetSize_) = firstSet;
    projection.block(0, firstSetSize_, 3, secondSetSize_) = secondSet;

    return projection;
}

double CoaxialityConstraint::getMinValue() const
{
    return minValue_;
}

void CoaxialityConstraint::setMinValue(double minValue)
{
    minValue_ = minValue;
}

double CoaxialityConstraint::getMaxValue() const
{
    return maxValue_;
}

void CoaxialityConstraint::setMaxValue(double maxValue)
{
    maxValue_ = maxValue;
}

unsigned int CoaxialityConstraint::getFirstSetSize() const
{
    return firstSetSize_;
}

void CoaxialityConstraint::setFirstSetSize(unsigned int value)
{
    firstSetSize_ = value;
}

unsigned int CoaxialityConstraint::getSecondSetSize() const
{
    return secondSetSize_;
}

void CoaxialityConstraint::setSecondSetSize(unsigned int value)
{
    secondSetSize_ = value;
}

ExtendedTrimesh *CoaxialityConstraint::getMesh() const
{
    return mesh;
}

void CoaxialityConstraint::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

std::vector<std::vector<unsigned int> > CoaxialityConstraint::getFirstSetOutlines() const
{
    return firstSetOutlines;
}

void CoaxialityConstraint::setFirstSetOutlines(const std::vector<std::vector<unsigned int> > &value)
{
    firstSetOutlines = value;
}

std::vector<std::vector<unsigned int> > CoaxialityConstraint::getSecondSetOutlines() const
{
    return secondSetOutlines;
}

void CoaxialityConstraint::setSecondSetOutlines(const std::vector<std::vector<unsigned int> > &value)
{
    secondSetOutlines = value;
}

unsigned int CoaxialityConstraint::getFirstSetSeedLoop() const
{
    return firstSetSeedLoop;
}

void CoaxialityConstraint::setFirstSetSeedLoop(unsigned int value)
{
    firstSetSeedLoop = value;
}

unsigned int CoaxialityConstraint::getSecondSetSeedLoop() const
{
    return secondSetSeedLoop;
}

void CoaxialityConstraint::setSecondSetSeedLoop(unsigned int value)
{
    secondSetSeedLoop = value;
}

//RepulsionConstraint::RepulsionConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions):
//    Constraint(idI, weight)
//{

//}

//void RepulsionConstraint::project(const Matrix3X &positions, Matrix3X &projections) const
//{

//}

//void RepulsionConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const
//{

//}

//Matrix3X RepulsionConstraint::getProjection(const Matrix3X &positions) const
//{

//}




///////////////////////////////////////////////////////////////////////////////
} // namespace ShapeOp
///////////////////////////////////////////////////////////////////////////////
