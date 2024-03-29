///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014 Sofien Bouaziz <sofien.bouaziz@gmail.com>, Bailin Deng <bailin.deng@epfl.ch>, Mario Deuss <mario.deuss@epfl.ch>
// Copyright (C) 2014 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#ifndef CONSTRAINT_H
#define CONSTRAINT_H
///////////////////////////////////////////////////////////////////////////////
#include "Types.h"
#include <memory>
#include <cmath>
#include <extendedtrimesh.h>

#ifndef M_PI
/** Defining pi.*/
#define M_PI 3.14159265358979323846
#endif
///////////////////////////////////////////////////////////////////////////////
/** \file
This file containts all the constraints of the ShapeOp libary.*/
///////////////////////////////////////////////////////////////////////////////
class ExtendedTrimesh;

namespace ShapeOp {

///////////////////////////////////////////////////////////////////////////////
/** \brief Base class of any constraints. This class defines the interface of a ShapeOp constraint.*/
class SHAPEOP_API Constraint {
 public:

  /** \brief Creates a constraint from a string type, a number of indices, a weight and the initial point positions.
   \param ConstraintType One of the following:
     - "EdgeStrain", see #ShapeOp::EdgeStrainConstraint
     - "TriangleStrain", see #ShapeOp::TriangleStrainConstraint
     - "TetrahedronStrain", see #ShapeOp::TetrahedronStrainConstraint
     - "Area", see #ShapeOp::AreaConstraint
     - "Volume", see #ShapeOp::VolumeConstraint
     - "Bending", see #ShapeOp::BendingConstraint
     - "Closeness", see #ShapeOp::ClosenessConstraint
     - "Line", see #ShapeOp::LineConstraint
     - "Plane", see #ShapeOp::PlaneConstraint
     - "Circle", see #ShapeOp::CircleConstraint
     - "Sphere", see #ShapeOp::SphereConstraint
     - "Similarity", see #ShapeOp::SimilarityConstraint with scaling
     - "Rigid", see #ShapeOp::SimilarityConstraint without scaling
     - "Rectangle", see #ShapeOp::RectangleConstraint
     - "Parallelogram", see #ShapeOp::ParallelogramConstraint
     - "Laplacian", see #ShapeOp::UniformLaplacianConstraint
     - "LaplacianDisplacement", see #ShapeOp::UniformLaplacianConstraint of deformation
   \param idI A vector of indices of the vertices to be constrained.
   \param weight The weight of the constraint to be added relative to the other constraints.
   \param positions The positions of all the n vertices stacked in a 3 by n matrix.
   \return A std::shared pointer to the Constraint, which is empty/null if failed.
   */
  static std::shared_ptr<Constraint> shapeConstraintFactory(const std::string &ConstraintType, const std::vector<int> &idI, Scalar weight, const Matrix3X &positions);
  /** \brief Constraint constructor.
   \param idI A vector of indices of the vertices to be constrained.
   \param weight The weight of the constraint to be added relative to the other constraints.
  */
  Constraint(const std::vector<int> &idI, Scalar weight);
  virtual ~Constraint() {}
  /** \brief Find the closest, in the least-squre sense, configuration from the input positions that satisfy the constraint.
      \param positions The positions of all the n vertices stacked in a 3 by n matrix.
      \param projections The projections of the vertices involved in the constraint.
  */
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const = 0;
  /** \brief Add the constraint to the linear system.
      \param[out] triplets A vector of triplets each representing an entry in a sparse matrix.
      \param[in,out] idO In: The first row index of the constraint in the sparse matrix. Out: The last row index plus one.
  */
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const = 0;

  virtual Matrix3X getProjection(const Matrix3X &positions) const = 0;

  /** \brief Number of indices of vertices involved in the constraint. */
  virtual std::size_t nIndices() const { return idI_.size(); }

  std::vector<int> getIds() { return idI_; }
  Scalar getWeight() const;
  void setWeight(const Scalar &weight);

  int getIdO() const;

protected:
  /** \brief ids of the vertices involved in this constraint.*/
  std::vector<int> idI_;
  /** \brief weight for the constraint.*/
  Scalar weight_;
  /** \brief location of this constraint in the linear system.*/
  mutable int idO_;

};
///////////////////////////////////////////////////////////////////////////////
/** \brief Edge strain constraint. Constrains the distance between two points to a range. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API EdgeStrainConstraint : public Constraint {
 public:
  /** \brief Constraint constructor. The target length is set to the distance of the two vertices in the parameter positions.
      The parameters rangeMin and rangeMax can be used to specify a target range for the distance (equivalent to edge length) [rangeMin*distance,rangeMax*distance]
    \param idI are two indices of the vertices of the edge.
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param rangeMin The factor to determine the minimal distance from the target length: rangeMin*distance
    \param rangeMax The factor to determine the maximal distance from the target length: rangeMax*distance
  */
  EdgeStrainConstraint(const std::vector<int> &idI,
                       Scalar weight,
                       const Matrix3X &positions,
                       Scalar rangeMin = 1.0,
                       Scalar rangeMax = 1.0);
  virtual ~EdgeStrainConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new edge length.*/
  void setEdgeLength(Scalar length);
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  Scalar rest_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A mesh-independent triangle strain-limiting constraint. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API TriangleStrainConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI are three indices of the vertices of the triangle
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  \param rangeMin The minimal strain.
  \param rangeMax The maximal strain.
  */
  TriangleStrainConstraint(const std::vector<int> &idI,
                           Scalar weight,
                           const Matrix3X &positions,
                           Scalar rangeMin = 1.0,
                           Scalar rangeMax = 1.0);
  virtual ~TriangleStrainConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  Matrix22 rest_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A mesh-independent tetrahedron strain-limiting constraint. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API TetrahedronStrainConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI are four indices of the vertices of the tetrahedron
  \param idI are three indices of the vertices of the triangle
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  \param rangeMin The minimal strain.
  \param rangeMax The maximal strain.
  */
  TetrahedronStrainConstraint(const std::vector<int> &idI,
                              Scalar weight,
                              const Matrix3X &positions,
                              Scalar rangeMin = 1.0,
                              Scalar rangeMax = 1.0);
  virtual ~TetrahedronStrainConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  Matrix33 rest_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Area constraint. Limits the area of a triangle to a range. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API AreaConstraint : public Constraint {
 public:
  /** \brief Constraint constructor. The target area is the area spanned by three vertices in the parameter positions.
      The parameters rangeMin and rangeMax can be used to specify a target range for the area [rangeMin* target_area,rangeMax* target_area]
    \param idI are three indices of the vertices forming the constrained triangle
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param rangeMin The factor to determine the minimal area: rangeMin*target_area.
    \param rangeMax The factor to determine the maximal area: rangeMax*target_area.
  */
  AreaConstraint(const std::vector<int> &idI,
                 Scalar weight,
                 const Matrix3X &positions,
                 Scalar rangeMin = 1.0,
                 Scalar rangeMax = 1.0);
  virtual ~AreaConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  Matrix22 rest_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Volume constraint. Limits the volume of a tetrahedron to a range. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API VolumeConstraint : public Constraint {
 public:
  /** \brief Constraint constructor. The target volume is the volume spanned by four vertices in the parameter positions.
      The parameters rangeMin and rangeMax can be used to specify a target range for the volume [rangeMin*target_volume,rangeMax*target_volume]
    \param idI are four indices of the vertices of the tetrahedron
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param rangeMin The factor to determine the minimal volume: rangeMin*target_volume.
    \param rangeMax The factor to determine the maximal volume: rangeMax*target_volume.
  */
  VolumeConstraint(const std::vector<int> &idI,
                   Scalar weight,
                   const Matrix3X &positions,
                   Scalar rangeMin = 1.0,
                   Scalar rangeMax = 1.0);
  virtual ~VolumeConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  Matrix33 rest_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Bending constraint. Limits the bending between two neighboring triangles. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API BendingConstraint : public Constraint {
 public:
  /** \brief Constraint constructor. The target bend is set to the bend between the triangles spanned by four vertices in the parameter positions. The parameters rangeMin and rangeMax can be used to specify a target range for the bend [rangeMin*target_bend,rangeMax*target_bend]
    The bending constraint applies to two neighboring triangles sharing an edge
    \param idI are four indices of the vertices of the two triangles ordered as follows:
     <pre>
      |        id2        |
      |       /   \       |
      |     id0---id1     |
      |       \   /       |
      |        id3        |
     </pre>
    \param idI are four indices of the vertices of the tetrahedron
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param rangeMin The factor to determine the minimal bend: rangeMin*target_bend.
    \param rangeMax The factor to determine the maximal bend: rangeMax*target_bend.
  */
  BendingConstraint(const std::vector<int> &idI,
                    Scalar weight,
                    const Matrix3X &positions,
                    Scalar rangeMin = 1.0,
                    Scalar rangeMax = 1.0);
  virtual ~BendingConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new range minimum.*/
  void setRangeMin(Scalar rMin) { rangeMin_ = rMin; }
  /** \brief Set a new range maximum.*/
  void setRangeMax(Scalar rMax) { rangeMax_ = rMax; }
 private:
  VectorX w_;
  Scalar n_;
  Scalar rangeMin_;
  Scalar rangeMax_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Closeness constraint. Constrains a vertex to a position in space. Projects onto a given rest position.*/
class SHAPEOP_API ClosenessConstraint : public Constraint {
 public:
  /** \brief Constraint constructor. The target is the position of vertex idI in the initial positions given in the constructor.
    \param idI contains the one index of the vertex that wants to be close to a given target.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  ClosenessConstraint(const std::vector<int> &idI,
                      Scalar weight,
                      const Matrix3X &positions);
  virtual ~ClosenessConstraint() override {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X & /*positions*/, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new closeness position.*/
  void setPosition(const Vector3 &position);
  /** \brief Get the closeness position.*/
  Vector3 getPosition() const;
 private:
  Vector3 rest_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Line constraint. Constrains a set of vertices to lie on the same line. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API LineConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI Indices of vertices to be projection onto on a line.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  LineConstraint(const std::vector<int> &idI,
                 Scalar weight,
                 const Matrix3X &positions);
  virtual ~LineConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
 private:
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Plane constraint. Constrains a set of vertices to lie on the same plane. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API PlaneConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI Indices of vertices to be projection onto on a plane.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  PlaneConstraint(const std::vector<int> &idI,
                  Scalar weight,
                  const Matrix3X &positions);
  virtual ~PlaneConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
 private:
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Circle constraint. Constrains a set of vertices to lie on the same circle. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API CircleConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI Indices of vertices to be projection onto on a circle.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  CircleConstraint(const std::vector<int> &idI,
                   Scalar weight,
                   const Matrix3X &positions);
  virtual ~CircleConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
 private:
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Sphere constraint. Constrains a set of vertices to lie on a sphere. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API SphereConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI Indices of vertices to be projection onto on a sphere.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  SphereConstraint(const std::vector<int> &idI,
                   Scalar weight,
                   const Matrix3X &positions);
  virtual ~SphereConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
 private:
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Similarity or Rigid constraint. Perserves the relative location of a set of vertices. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API SimilarityConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
    By default the shape to match is given by the initial positions of the vertices involved.
    \param idI A vector of indices of the vertices to be constrained.
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param scaling If true, the matching is only up to scale, if false not.
    \param rotate If true the constraint minimizes over all possible rotations of the matchings.
    \param flip If true the constraint minimizes by also flipping the matchings.
  */
  SimilarityConstraint(const std::vector<int> &idI,
                       Scalar weight,
                       const Matrix3X &positions,
                       bool isNotRigid = true,
                       bool rotate = true,
                       bool flip = true);
  virtual ~SimilarityConstraint() override {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief A set of new shapes to match to. The SimilarityConstraint will choose and project onto the closest.
      \param shapes a vector of shapes. Each shape consists of a 3*nIndices() Matrix representing a set of points.
  */
  void setShapes( const std::vector<Matrix3X> &shapes );
  bool isNotRigid() const;
  void setIsNotRigid(bool isNotRigid);

private:
  void testCandidates(Scalar &min_error) const;
  mutable Matrix3X input;
  mutable Matrix3X candidate;
  mutable Matrix3X output;
  std::vector<Matrix3X>  shapes_;
  bool scaling_;
  bool rotate_;
  bool flip_;
  mutable Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permutation_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> rotateMatrix_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> flipMatrix_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Rectangle constraint. Constrains the four vertices of a quad to be a rectangle. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API RectangleConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
    \param idI Indices of vertices to be projection onto on a sphere.
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  RectangleConstraint(const std::vector<int> &idI,
                      Scalar weight,
                      const Matrix3X &positions);
  virtual ~RectangleConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief ParallelogramConstraint constraint. Constrains the four vertices of a quad to be a parallelogram. See \cite Bouaziz2012 for more details.*/
class SHAPEOP_API ParallelogramConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
    \param idI Indices of vertices to be projection onto on a sphere.
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  ParallelogramConstraint(const std::vector<int> &idI,
                          Scalar weight,
                          const Matrix3X &positions);
  virtual ~ParallelogramConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
 private:
  MatrixT<8, 8> parallelogramMatrix;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Uniform Laplacian Constraint. Minimizes the uniform laplacian respectively the uniform laplacian of displacements.*/
class SHAPEOP_API UniformLaplacianConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI The first index corresponds to the central vertex, while the remaining ones correspond to the one-ring neighborhood.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  \param displacement_lap If false the laplacian is minimized, if true, the laplacian of deformations w.r.t. initial positions.
  */
  UniformLaplacianConstraint(const std::vector<int> &idI,
                             Scalar weight,
                             const Matrix3X &positions,
                             bool displacement_lap);
  virtual ~UniformLaplacianConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X & /*positions*/, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  Vector3 rest() const;
  void setRest(const Vector3 &rest);

  virtual std::size_t nIndices() const override;
  bool getDisplacement() const;
  void setDisplacement(bool value);

private:
  bool displacement;
  Vector3 rest_;
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Cotangent Laplacian Constraint. Minimizes the uniform laplacian respectively the uniform laplacian of displacements.*/
class SHAPEOP_API CotangentLaplacianConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI The first index corresponds to the central vertex, while the remaining ones correspond to the one-ring neighborhood.
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  \param displacement_lap If false the laplacian is minimized, if true, the laplacian of deformations w.r.t. initial positions.
  */
  CotangentLaplacianConstraint(const std::vector<int> &idI,
                             Scalar weight,
                             const Matrix3X &positions,
                             bool displacement_lap);
  virtual ~CotangentLaplacianConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X & /*positions*/, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  Vector3 rest() const;
  void setRest(const Vector3 &rest);

  virtual std::size_t nIndices() const override;

  bool getDisplacement() const;
  void setDisplacement(bool value);

private:

  bool displacement;
  mutable VectorX cotWeights;
  Vector3 rest_;
  mutable Matrix3X input;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief Angle range constraint. Constrains the angle between the two lines formed by three points to a range. See \cite Deng2015 for more details.*/
class SHAPEOP_API AngleConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
    \param idI are three indices of the vertices [v0,v1,v2]. The constrained angle is the one spanned between v1-v0 and v2-v0:
     <pre>
      |         id0       |
      |        /  |       |
      |       /   |       |
      |    id1    id2     |
     </pre>
    \param weight The weight of the constraint to be added relative to the other constraints.
    \param positions The positions of all the n vertices stacked in a 3 by n matrix.
    \param minAngle The minimum angle (in radian, between 0 and PI).
    \param maxAngle The maximum angle (in radian, between 0 and PI).
  */
  AngleConstraint(const std::vector<int> &idI,
                  Scalar weight,
                  const Matrix3X &positions,
                  Scalar minAngle = 0.0,
                  Scalar maxAngle = M_PI);
  virtual ~AngleConstraint() {}
  /** \brief Find the closest configuration from the input positions that satisfy the angle constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  /** \brief Set a new minimum angle (in radian).*/
  void setMinAngle(Scalar minAngle);
  /** \brief Set a new maximum angle (in radian).*/
  void setMaxAngle(Scalar maxAngle);
  Scalar minAngle() const;
  Scalar maxAngle() const;

private:
  Scalar minAngle_;
  Scalar maxAngle_;
  Scalar minAngleCos_;
  Scalar maxAngleCos_;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief Orientation constraint. Constrains the points to lie on the plane passing through the origin with user defined normal.*/
class SHAPEOP_API OrientationConstraint : public Constraint {
public:
public:
  OrientationConstraint(const std::vector<int> &idI,
                      Scalar weight,
                      const Matrix3X &positions,
                      Vector3 normal = Vector3(0.0,0.0,1.0));
  virtual ~OrientationConstraint() {}
  virtual void project(const Matrix3X & positions, Matrix3X &projections) const override final;
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  void setOrientation(const Vector3 &normal);

private:
  Vector3 normal_;
  mutable Matrix3X input;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief Regression plane orientation constraint. Constrains the points general orientation (given by regression plane) to adapt to a user defined normal */
class SHAPEOP_API RegressionPlaneOrientationConstraint : public Constraint {
public:
  RegressionPlaneOrientationConstraint(const std::vector<int> &idI,
                      Scalar weight,
                      const Matrix3X &positions,
                      Vector3 normal = Vector3(0.0,0.0,1.0));
  virtual ~RegressionPlaneOrientationConstraint() {}
  virtual void project(const Matrix3X & positions, Matrix3X &projections) const override final;
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
  void setOrientation(const Vector3 &normal);

    private:
      Vector3 normal_;
      mutable Matrix3X input;
};


///////////////////////////////////////////////////////////////////////////////
/** \brief Regression plane orientation constraint. Constrains the points general orientation (given by regression plane) to adapt to a user defined normal */
class SHAPEOP_API PolylineLengthConstraint : public Constraint {
public:
    PolylineLengthConstraint(const std::vector<int> &idI,
                        Scalar weight,
                        const Matrix3X &positions,
                        const Scalar minRange,
                        const Scalar maxRange);
    virtual ~PolylineLengthConstraint() {}
    virtual void project(const Matrix3X & positions, Matrix3X &projections) const override final;
    virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
    virtual Matrix3X getProjection(const Matrix3X &positions) const override;
    Scalar getRangeMin() const;
    void setRangeMin(const Scalar &value);

    Scalar getRangeMax() const;
    void setRangeMax(const Scalar &value);

private:
    mutable Matrix3X input;
    Scalar rangeMin_;
    Scalar rangeMax_;
    Scalar rest_;
};
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/** \brief Regression plane orientation constraint. Constrains the points general orientation (given by regression plane) to adapt to a user defined normal */
class SHAPEOP_API RectangleFitConstraint : public Constraint {
public:
    RectangleFitConstraint(const std::vector<int> &idI,
                        Scalar weight,
                        const Matrix3X &positions);
    virtual ~RectangleFitConstraint() {}
    virtual void project(const Matrix3X & positions, Matrix3X &projections) const override final;
    virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
    virtual Matrix3X getProjection(const Matrix3X &positions) const override;

private:
    mutable Matrix3X input;

};

///////////////////////////////////////////////////////////////////////////////
/** \brief A mesh-independent triangle stretch-minimizing constraint. See \cite Bouaziz2014 for more details.*/
class SHAPEOP_API EquilateralConstraint : public Constraint {
 public:
  /** \brief Constraint constructor.
  \param idI are three indices of the vertices of the triangle
  \param weight The weight of the constraint to be added relative to the other constraints.
  \param positions The positions of all the n vertices stacked in a 3 by n matrix.
  */
  EquilateralConstraint(const std::vector<int> &idI,
                           Scalar weight,
                           const Matrix3X &positions);
  virtual ~EquilateralConstraint() override{}
  /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
  virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
  /** \brief Add the constraint to the linear system.*/
  virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
  virtual Matrix3X getProjection(const Matrix3X &positions) const override;
private:
  mutable Matrix3X input;
};

///////////////////////////////////////////////////////////////////////////////
class SHAPEOP_API SimilarLengthConstraint : public Constraint {
    public:
        SimilarLengthConstraint(const std::vector<int> &idI,
                            Scalar weight,
                            const Matrix3X &positions,
                            Scalar minProportion_ = 1.0,
                            Scalar maxProportion_ = 1.0);
        virtual ~SimilarLengthConstraint() override;
        /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
        virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
        /** \brief Add the constraint to the linear system.*/
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        virtual Matrix3X getProjection(const Matrix3X &positions) const override;
        unsigned int getLinePointsCardinality1() const;
        void setLinePointsCardinality1(unsigned int value);
        unsigned int getLinePointsCardinality2() const;
        void setLinePointsCardinality2(unsigned int value);

        double getMinProportion() const;
        void setMinProportion(double value);

        virtual std::size_t nIndices() const override;
        double getMaxProportion() const;
        void setMaxProportion(double value);

private:
        mutable Matrix3X input;
        unsigned int linePointsCardinality1;
        unsigned int linePointsCardinality2;
        double minProportion_, maxProportion_;
};

class SHAPEOP_API NonUniformScalingConstraint : public Constraint {
    public:
        NonUniformScalingConstraint(const std::vector<int> &idI,
                            Scalar weight,
                            const Matrix3X &positions,
                            Scalar minProportion = 1.0,
                            Scalar maxProportion = 1.0,
                            int l1StartIndex = 0,
                            int l1EndIndex = 1,
                            int l2StartIndex = 2,
                            int l2EndIndex = 3,
                            bool l1directed = false,
                            bool l2directed = false,
                            Vector3 direction1 = {0,0,1},
                            Vector3 direction2 = {0,0,1});
        virtual ~NonUniformScalingConstraint() override;
        /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
        virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
        /** \brief Add the constraint to the linear system.*/
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;
        virtual Matrix3X getProjection(const Matrix3X &positions) const override;
        double getMinProportion() const;
        void setMinProportion(double minProportion);
        double getMaxProportion() const;
        void setMaxProportion(double maxProportion);
        int getL1StartIndex() const;
        void setL1StartIndex(int l1StartIndex);
        int getL1EndIndex() const;
        void setL1EndIndex(int l1EndIndex);
        int getL2StartIndex() const;
        void setL2StartIndex(int l2StartIndex);
        int getL2EndIndex() const;
        void setL2EndIndex(int l2EndIndex);

        Vector3 getL1direction() const;
        void setL1direction(const Vector3 &l1direction);

        Vector3 getL2direction() const;
        void setL2direction(const Vector3 &value);

        bool getL1directed() const;
        void setL1directed(bool l1directed);

        bool getL2directed() const;
        void setL2directed(bool l2directed);

private:
        mutable Matrix3X input;
        Vector3 l1direction_, l2direction;
        double minProportion_, maxProportion_;
        int l1StartIndex_, l1EndIndex_;
        int l2StartIndex_, l2EndIndex_;
        bool l1directed_, l2directed_;
};


class SHAPEOP_API CoaxialityConstraint : public Constraint {
    public:
        const int EDGE_FLAGGED = 981;
        const int TRIANGLE_FLAGGED = 982;
        const int TRIANGLE_INSERTED = 983;

        CoaxialityConstraint(const std::vector<int> &idI,
                            Scalar weight,
                            const Matrix3X &positions,
                            Scalar minValue = 1.0,
                            Scalar maxValue = 1.0,
                            unsigned int firstSetSize_ = 0,
                            unsigned int secondSetSize_ = 0);

        virtual ~CoaxialityConstraint() override;
        /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
        virtual void project(const Matrix3X &positions, Matrix3X &projections) const override final;
        /** \brief Add the constraint to the linear system.*/
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;

        std::pair<unsigned int, std::vector<Vector3>> sphere_line_intersection (Vector3 p1, Vector3 p2, Vector3 p3, double r, bool inSegment) const;
        Matrix3X extractSkeleton(std::vector<unsigned int> seedLoop) const;
        virtual Matrix3X getProjection(const Matrix3X &positions) const override final;
        double getMinValue() const;
        void setMinValue(double minValue);
        double getMaxValue() const;
        void setMaxValue(double maxValue);
        unsigned int getFirstSetSize() const;
        void setFirstSetSize(unsigned int value);
        unsigned int getSecondSetSize() const;
        void setSecondSetSize(unsigned int value);
        ExtendedTrimesh *getMesh() const;
        void setMesh(ExtendedTrimesh *value);
        std::vector<std::vector<unsigned int> > getFirstSetOutlines() const;
        void setFirstSetOutlines(const std::vector<std::vector<unsigned int> > &value);
        std::vector<std::vector<unsigned int> > getSecondSetOutlines() const;
        void setSecondSetOutlines(const std::vector<std::vector<unsigned int> > &value);
        unsigned int getFirstSetSeedLoop() const;
        void setFirstSetSeedLoop(unsigned int value);
        unsigned int getSecondSetSeedLoop() const;
        void setSecondSetSeedLoop(unsigned int value);

private:
        mutable Matrix3X input;
        ExtendedTrimesh* mesh;
        std::vector<std::vector<unsigned int> > firstSetOutlines;
        std::vector<std::vector<unsigned int> > secondSetOutlines;
        unsigned int firstSetSeedLoop, secondSetSeedLoop;
        double minValue_, maxValue_;
        unsigned int firstSetSize_, secondSetSize_;
};



} // namespace ShapeOp

#endif // CONSTRAINT_H
///////////////////////////////////////////////////////////////////////////////
