#ifndef HT_MATH_H
#define HT_MATH_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>
#include <vector>

namespace ik {

	//=== types =======================================================//

	// 3d vector types
	typedef Eigen::Vector3f Vec3f;
	typedef Eigen::Vector3d Vec3d;

	// 4d vector types
	typedef Eigen::Vector4f Vec4f;
	typedef Eigen::Vector4d Vec4d;

	// dynamic vector types
	typedef Eigen::VectorXf VecXf;
	typedef Eigen::VectorXd VecXd;

	// 3x3 matrix types
	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Mat3f;
	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3d;

	// 4x4 matrix types
	typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Mat4f;
	typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Mat4d;

	// dynamic matrix types
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatXf;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatXd;

	// affine transformation types
	typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::RowMajor> Transform3f;
	typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::RowMajor> Transform3d;

	// aligned stl vector for eigen3 (substitute with c++11 typedef alias when gcc supports it)
	template<typename T>
	struct aligned_vector {
		typedef std::vector<T, Eigen::aligned_allocator<T> > type;
	};

	// aligned stl map for eigen3 (substitute with c++11 typedef alias when gcc supports it)
	template<typename K, typename V>
	struct aligned_map {
		typedef std::map<K, V, std::less<K>,
			Eigen::aligned_allocator<std::pair<const K, V> > > type;
	};

	//=== functions ===================================================//

	// conversion between VecXf and std::vector
	std::vector<float> toStdVector(const VecXf &v);
	VecXf toVecXf(const std::vector<float> &v);

	// euler angle conversion functions
	Vec3f toEulerAngles(const Mat3f &m, int e1, int e2, int e3);
	Vec3f toEulerAngles(const Mat4f &m, int e1, int e2, int e3);
	Mat3f fromEulerAngles(const Vec3f &v, int e1, int e2, int e3);
	Mat3f fromEulerAngles(float a, float b, float c, int e1, int e2, int e3);

	// matrix functions
	MatXf pinv(const MatXf &sym);
	MatXf transp(const MatXf &m);

	// maxvol
	MatXf maxvol(const MatXf &m, bool initLU = true);
	MatXf maxvol(const MatXf &m, std::vector<int> &indices, bool initLU = true);
	void maxvol2(
		const std::vector<MatXf> &matrices,
		const std::vector<int> &initPivot,
		std::vector<int> &indices);

	// matrix row pivoting stuff
	std::vector<int> pivotCustom(MatXf &diagonalBlockJacobian, bool pivot = true);
	std::vector<int> farthestPoints(MatXf &rowVectors, bool angleMetric = true);
	std::vector<int> fpoSubset(MatXf &rowVectors, int num, bool angleMetric = true);
	void rankingStuff(const MatXf &A, std::vector<int> &indices, MatXf &A_);

    // smooth interpolation [ t2  t1  x_1 ... x_n  n1  n2 ] subject to ks[is[...]]
    std::vector<float> interpolate(
        size_t n,
        float t2, float t1,
        float n1, float n2,
        const std::vector<float> &ks,
        const std::vector<size_t> &is);

} // namespace ik

#endif

