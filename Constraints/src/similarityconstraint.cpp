#include "similarityconstraint.h"

namespace ShapeOp{
SimilarityConstraint::SimilarityConstraint(const std::vector<int> &idI, double weight, const std::shared_ptr<Matrix3X> positions, bool scaling /*= true*/, bool rotate /*=true*/, bool flip /*=true*/) :
    Constraint(idI, weight), scaling_(scaling), rotate_(rotate), flip_(flip)
{
    assert(idI.size() >= 2);
    input = Matrix3X::Zero(3, idI.size());
    candidate = Matrix3X::Zero(3, idI.size());
    output = Matrix3X::Zero(3, idI.size());
    Matrix3X shape = Matrix3X::Zero(3, idI.size());
    for (int i = 0; i < static_cast<int>(idList.size()); ++i) shape.col(i) = positions->col(idList[i]);
    modelPoints = positions;
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

void SimilarityConstraint::project(std::shared_ptr<MatrixX3> projections)  const {
    for (int i = 0; i < static_cast<int>(idList.size()); ++i) input.col(i) = modelPoints->col(idList[i]);
    Vector3 mean_vector = input.rowwise().mean();
    input.colwise() -= mean_vector;

    double min_error = std::numeric_limits<double>::max();
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
    projections->block(this->idO, 0, input.cols(), 3) = output * weight;
}

    void SimilarityConstraint::testCandidates(double &min_error) const {
        for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
          candidate = Eigen::umeyama(shapes_[i] * permutation_, input, scaling_).block<3, 3>(0, 0) * shapes_[i] * permutation_;
          double error = (input - candidate).squaredNorm();
          if (error < min_error) {
            min_error = error;
            output = candidate;
          }
        }
    }

    void SimilarityConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
        this->idO = idO;
        int n_idx = static_cast<int>(idList.size());
        double coef1 = (1.0 - 1.0 / n_idx) * weight;
        double coef2 = -weight / n_idx;
        for (int i = 0; i < n_idx; ++i) {
          for (int j = 0; j < n_idx; ++j)
            triplets.push_back(Triplet(idO, idList[j], (i == j ? coef1 : coef2)));
          idO++;
        }
    }

    void SimilarityConstraint::setShapes(const std::vector<Matrix3X> &shapes) {
        shapes_ = shapes;
        for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
          Vector3 mean_vector = shapes_[i].rowwise().mean();
          shapes_[i].colwise() -= mean_vector;
        }
    }
}
