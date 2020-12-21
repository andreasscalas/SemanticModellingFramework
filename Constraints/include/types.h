#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/LU>
namespace ShapeOp {

    //Dense
    template < int Rows, int Cols, int Options = (Eigen::ColMajor | Eigen::AutoAlign) >
    using MatrixT = Eigen::Matrix<double, Rows, Cols, Options>; ///< A typedef of the dense matrix of Eigen.
    typedef MatrixT<2, 1> Vector2;								///< A 2d column vector.
    typedef MatrixT<2, 2> Matrix22;								///< A 2 by 2 matrix.
    typedef MatrixT<2, 3> Matrix23;								///< A 2 by 3 matrix.
    typedef MatrixT<3, 1> Vector3;								///< A 3d column vector.
    typedef MatrixT<3, 2> Matrix32;								///< A 3 by 2 matrix.
    typedef MatrixT<3, 3> Matrix33;								///< A 3 by 3 matrix.
    typedef MatrixT<3, 4> Matrix34;								///< A 3 by 4 matrix.
    typedef MatrixT<4, 1> Vector4;								///< A 4d column vector.
    typedef MatrixT<4, 4> Matrix44;								///< A 4 by 4 matrix.
    typedef MatrixT<3, Eigen::Dynamic> Matrix3X;				///< A 3 by n matrix.
    typedef MatrixT<Eigen::Dynamic, 3> MatrixX3;				///< A n by 3 matrix.
    typedef MatrixT<Eigen::Dynamic, 1> VectorX;					///< A nd column vector.
    typedef MatrixT<Eigen::Dynamic, Eigen::Dynamic> MatrixXX;	///< A n by m matrix.
    //Sparse
    template<int Options = Eigen::ColMajor>
    using SparseMatrixT = Eigen::SparseMatrix<double, Options>;	///< A typedef of the sparse matrix of Eigen.
    typedef SparseMatrixT<> SparseMatrix;
    typedef Eigen::Triplet<double>  Triplet;
}
#endif // TYPES_H
