#include "Math.h"

#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <omp.h>

#define NUM_THREADS 8

using namespace Eigen;

namespace ik {

//===========================================================================//

std::vector<float> toStdVector(const VecXf &v)
{
	std::vector<float> vec(v.data(), v.data() + v.size());
	return vec;
}

//===========================================================================//

VecXf toVecXf(const std::vector<float> &v)
{
	Eigen::Map<const VecXf> vec(&v[0], v.size());
	return VecXf(vec);
}

//===========================================================================//

Vec3f toEulerAngles(const Mat3f &m, int e1, int e2, int e3)
{
	return m.eulerAngles(e1,e2,e3);
}

//===========================================================================//

Vec3f toEulerAngles(const Mat4f &m, int e1, int e2, int e3)
{
	Mat3f m_ = m.block(0,0,3,3);
	return toEulerAngles(m_, e1, e2, e3);
}

//===========================================================================//

Mat3f fromEulerAngles(const Vec3f &v, int e1, int e2, int e3)
{
	return Mat3f(
		AngleAxisf(v[0], Vec3f::Unit(e1)) *
		AngleAxisf(v[1], Vec3f::Unit(e2)) *
		AngleAxisf(v[2], Vec3f::Unit(e3)) );
}

//===========================================================================//

Mat3f fromEulerAngles(float a, float b, float c, int e1, int e2, int e3)
{
	return fromEulerAngles(Vec3f(a, b, c), e1, e2, e3);
}

//===========================================================================//

MatXf pinv(const MatXf &sym)
{
	return sym.llt().solve(MatXf::Identity(sym.rows(), sym.rows()));
}

//===========================================================================//

MatXf transp(const MatXf &m)
{
	size_t r = m.rows();
	size_t c = m.cols();

	MatXf mT(c, r);

	omp_set_num_threads(NUM_THREADS);
#pragma omp parallel for
    for(int i = 0; i < r; ++i)
		for(size_t j = 0; j < c; ++j)
			mT(j,i) = m(i,j);

	return mT;
}

//===========================================================================//

MatXf maxvol(const MatXf &A, bool initLU)
{
	std::vector<int>_;
	return maxvol(A,_,initLU);
}

//===========================================================================//

MatXf maxvol(const MatXf &_A, std::vector<int> &indices, bool initLU)
{
	// matrix dimensions

	MatXf A = _A;

	std::cout<<"input matrix:"<<std::endl<<A<<std::endl;

	int n = A.rows();
	int m = A.cols();

	if(m >= n)
	{
		indices = std::vector<int>(n);
		for(int i = 0; i < n; ++i)
			indices[i] = i;

		return A;
	}

	// row pivoting initialization

	MatXf A_, B;

	if(initLU)
	{
		// LU decomposition of A with pivoting
		// for initialization of the submatrix
		Eigen::FullPivLU<MatXf> lu(A);

		// initialize permuted A_ and B
		A_ = lu.permutationP() * A;
		B  = A_ * A_.block(0,0,m,m).inverse();

		// initialize index vector with permutation indices
		indices = std::vector<int>(
			lu.permutationP().indices().data(),
			lu.permutationP().indices().data() + n);

		//return A_.block(0,0,m,m);
	}
	else
	{
		// custom or no pivoting
		A_ = A;

		if(indices.empty())
		{
			// no pivoting
			indices = std::vector<int>(n);
			for(int i = 0; i < n; ++i)
				indices[i] = i;
		}
		else
		{
			// predefined pivoting
			for(int i = 0; i < n; ++i)
			{
				int j = indices[i];
				if(j != i) A_.row(i) = A.row(j);
			}
		}

 		B = A_ * A_.block(0,0,m,m).inverse();
	}

	// iterative matrix re-ordering
	int iter=0;

	while(true)
	{
		// find index j of maximum absolute coefficient in all rows i >= m
		float Bmax = 0;
		int jmax = 0;
		int imax = 0;

		for(int i = m; i < n; ++i)
		{
#if 1
			// check if constraint is free
			bool taken = false;
			for(int k = 0; k < m; ++k)
			{
				int i_c = indices[i] / 3;
				int k_c = indices[k] / 3;

				if(i_c == k_c)
				{
					taken = true;
					break;
				}
			}
			if(taken)
				continue;
#endif

			// find index j of maximum absolute coefficient in row i
			int j = i;
			float Bij = B.row(i).cwiseAbs().maxCoeff(&j);

			if(Bij > Bmax)
			{
				Bmax = Bij;
				imax = i;
				jmax = j;
			}
		}

		std::cout << "[maxvol] iteration " << iter++
				  << ", Bmax = " << Bmax
				  << "[" << imax << "," << jmax << "]"
				  << std::endl;

		if(Bmax <= 1.0f)
			break;

		// swap rows i and j in the current solution
		A_.row(imax).swap(A_.row(jmax));
		std::swap(indices[imax], indices[jmax]);

		// update B
		const VecXf &ej = VecXf::Unit(n, jmax);
		const VecXf &ei = VecXf::Unit(n, imax);
		const RowVectorXf &ejT = VecXf::Unit(m, jmax);
		float Bval = B(imax, jmax);

		B = B - (B.col(jmax) - ej + ei) * (B.row(imax) - ejT) / Bval;
	}

#if 0
	{
		using namespace std;
		cout<<"submatrix:"<<endl<<A_.block(0,0,A_.cols(),A_.cols())<<endl;
		vector<bool> seld(indices.size()/3,false);
		for(int i = 0; i < m; ++i)
			seld[indices[i]/3] = true;
		int count_unique = 0;
		for(size_t i = 0; i < seld.size(); ++i)
			if(seld[i]) count_unique++;
		for(int i = 0; i < m; ++i){
			bool is_x = ( indices[i] == ( ( indices[i] / 3 ) * 3 + 0) );
			bool is_y = ( indices[i] == ( ( indices[i] / 3 ) * 3 + 1) );
			bool is_z = ( indices[i] == ( ( indices[i] / 3 ) * 3 + 2) );
			cout<<i+1<<":"<<indices[i]<<"("<<indices[i]/3<<")";
			cout<<(is_x?"x":"")<<(is_y?"y":"")<<(is_z?"z":"")<<" ";
		}
		cout<<endl;
		cout<<"unique constraints: " << count_unique << endl;
		cout << endl << "--" << endl << endl;
	}
#endif

	//rankingStuff(A,indices,A_);

	//{
	//MatXf ATA = A_.block(0,0,m,m).transpose() * A_.block(0,0,m,m) + MatXf::Identity(m,m);
	//Eigen::JacobiSVD<MatXf> svd(ATA);
	//float con = fabs(svd.singularValues()[0] / svd.singularValues()[m-1]);
	//std::cout << "determinant: " << ATA.determinant() << std::endl;
	//std::cout << "condition: " << con << std::endl;
	//}

	//{
	//MatXf ATA = A_.transpose() * A_ + MatXf::Identity(m,m);
	//Eigen::JacobiSVD<MatXf> svd(ATA);
	//float con = fabs(svd.singularValues()[0] / svd.singularValues()[m-1]);
	//std::cout << "determinant: " << ATA.determinant() << std::endl;
	//std::cout << "condition: " << con << std::endl;
	//}

	return A_.block(0,0,m,m);
}

void rankingStuff(const MatXf &A, std::vector<int> &indices, MatXf &A_)
{
	//------------------------------------------------------------------

	int m = A.cols();

	{
		using namespace std;
		cout<<"input matrix:"<<endl<<A<<endl;
		vector<bool> seld(indices.size()/3,false);
		for(int i = 0; i < m; ++i)
			seld[indices[i]/3] = true;
		cout<<"selected (before ranking): ";
		for(size_t i = 0; i < seld.size(); ++i)
			cout << seld[i];
		cout<<endl;
		for(int i = 0; i < m; ++i)
			cout<<indices[i]<<" ";
		cout<<endl;
	}

	//------------------------------------------------------------------

	{
		using namespace std;
		cout << "submatrix before ranking:" << endl << A_.block(0,0,m,m) << endl;
		MatXf A_block = A_.block(0,0,m,m);

		// max distance, max angle pivot
		//vector<int> shuffle1 = farthestPoints(A_block, false); // distance
		//vector<int> shuffle2 = farthestPoints(A_block); // angle
		//vector<int> shuffle(shuffle1.size());
		//for(size_t i = 0; i < shuffle.size(); ++i)
			//shuffle[i] = shuffle1[shuffle2[i]];

		// max angle, max distance pivot
		//vector<int> shuffle1 = farthestPoints(A_block); // angle
		//vector<int> shuffle2 = farthestPoints(A_block, false); // distance
		//vector<int> shuffle(shuffle1.size());
		//for(size_t i = 0; i < shuffle.size(); ++i)
			//shuffle[i] = shuffle1[shuffle2[i]];

		// max distance pivot
		//vector<int> shuffle = farthestPoints(A_block, false); // distance

		// max angle pivot
		//vector<int> shuffle = farthestPoints(A_block); // angle

		// farthes point shuffle
		//vector<int> indices_copy = indices;
		//for(int i = 0; i < m; ++i)
			//indices[i] = indices_copy[shuffle[i]];
		//cout << endl;

		// lu pivot + shuffle
		//Eigen::FullPivLU<MatXf> lu(A_block);
		//std::vector<int> shuffle = std::vector<int>(
				//lu.permutationP().indices().data(),
				//lu.permutationP().indices().data() + m);
		//std::vector<int> indices_copy = indices;
		//for(int i = 0; i < m; ++i)
			//indices[shuffle[i]] = indices_copy[i];
		//A_block = lu.permutationP() * A_block;

		// fpo pivot + shuffle
		int numrows = A_block.rows();
		int selrows = 15;
		vector<int> subset = fpoSubset(A_block, selrows);
		vector<bool> select(A_block.rows());
		for(size_t i = 0; i < subset.size(); ++i){
			std::cout << "subset element " << i << ": " << subset[i] << std::endl;
			select[subset[i]] = true;
		}
		vector<int> otherset;
		for(size_t i = 0; i < select.size(); ++i)
			if(!select[i]) otherset.push_back(i);
		MatXf A_block_new(A_block.rows(), A_block.cols());
		for(size_t i = 0; i < subset.size(); ++i)
			A_block_new.row(i) = A_block.row(subset[i]);
		for(size_t i = 0; i < otherset.size(); ++i)
			A_block_new.row(i+selrows) = A_block.row(otherset[i]);
		A_block = A_block_new;
		std::vector<int> indices_copy = indices;
		for(int i = 0; i < selrows; ++i)
			indices[i] = indices_copy[subset[i]];
		for(int i = selrows; i < numrows; ++i)
			indices[i] = indices_copy[otherset[i-selrows]];

		A_.block(0,0,m,m) = A_block;
		cout << "submatrix after ranking:" << endl << A_block << endl;
	}

	//------------------------------------------------------------------

	{
		using namespace std;
		vector<bool> seld(indices.size()/3,false);
		for(int i = 0; i < m; ++i)
			seld[indices[i]/3] = true;
		cout<<"selected (after ranking): ";
		for(size_t i = 0; i < seld.size(); ++i)
			cout << seld[i];
		cout<<endl;
		for(int i = 0; i < m; ++i)
			cout<<indices[i]<<"("<<indices[i]/3<<")"<<" ";
		cout<<endl;
		cout << endl << "--" << endl << endl;
	}

	//------------------------------------------------------------------
}

//===========================================================================//

void maxvol2(
	const std::vector<MatXf> &matrices,
	const std::vector<int> &initPivot,
	std::vector<int> &indices)
{
	// initialize

	if(matrices.empty())
		return;

	int rows = matrices[0].rows();
	int cols = matrices[0].cols();

	if(cols >= rows)
		return;

	std::vector<MatXf> As(matrices.size(), MatXf::Zero(rows, cols));
	std::vector<MatXf> Bs(matrices.size(), MatXf::Zero(rows, cols));

	for(size_t m = 0; m < matrices.size(); ++m)
	{
		for(int i = 0; i < rows; ++i)
		{
			int j = initPivot[i];
			As[m].row(i) = matrices[m].row(j);
		}

		Bs[m] = As[m] * As[m].block(0,0,cols,cols).inverse();
	}

	indices = initPivot;

	// iterate
	int iter = 0;
	float Cprev = std::numeric_limits<float>::max();

	while(true)
	{
		// compute sum of absolute values in Bs
		MatXf C = MatXf::Zero(rows, cols);
		for(size_t m = 0; m < matrices.size(); ++m)
			C += Bs[m].array().abs().matrix();
		std::cout << C << std::endl;

		// find index j of maximum absolute coefficient in all rows i
		float Cmax = 0;
		int jmax = 0;
		int imax = 0;

#if 1

		// rowwise max element
		for(int i = cols; i < rows; ++i)
		{
#if 0

			// check if constraint is free
			bool taken = false;
			for(int k = 0; k < cols; ++k)
			{
				int i_c = indices[i] / 3;
				int k_c = indices[k] / 3;

				if(i_c == k_c)
				{
					taken = true;
					break;
				}
			}
			if(taken)
				continue;

#endif

			// find index j of maximum absolute coefficient in row i
			for(int j = 0; j < cols; ++j)
			{
				bool isnull = false;

				for(size_t k = 0; k < matrices.size(); ++k)
				{
					if(fabs(Bs[k](i,j)) < 0.01f)
					{
						isnull = true;
						break;
					}
				}

				if(isnull)
					continue;

				float Cij = C(i,j);

				if(Cij > Cmax)
				{
					Cmax = Cij;
					imax = i;
					jmax = j;
				}
			}
		}

#else

		// blockwise max element
		Cmax = C.block(cols, 0, rows-cols, cols).maxCoeff(&imax, &jmax);
		imax += cols; // offset introduced by only considering lower block

#endif

		std::cout << "[multi-maxvol] iteration " << iter++
				  << ", Cmax/N = " << Cmax/matrices.size()
				  << "[" << imax << "," << jmax << "]" << std::endl;

		bool end = ( ( Cmax / matrices.size() ) <= 1.0f );

		if(Cprev < Cmax / matrices.size())
			end = true;

		Cprev = Cmax / matrices.size();

		for(size_t m = 0; m < matrices.size(); ++m)
		{
			// prevent division by zero
			if(fabs(Bs[m](imax,jmax)) < 0.01f)
				end = true;
		}

		// no more swaps necessary/beneficial
		if(end) break;

		// swap row indices imax and jmax
		std::swap(indices[imax], indices[jmax]);

		// update As and Bs
		for(size_t m = 0; m < matrices.size(); ++m)
		{
			// swap rows imax and jmax in the current solution
			As[m].row(imax).swap(As[m].row(jmax)); // not used

			// update B
			const VecXf &ej = VecXf::Unit(rows, jmax);
			const VecXf &ei = VecXf::Unit(rows, imax);
			const RowVectorXf &ejT = VecXf::Unit(cols, jmax);
			float Bval = Bs[m](imax, jmax);
			Bs[m] = Bs[m] - (Bs[m].col(jmax) - ej + ei) * (Bs[m].row(imax) - ejT) / Bval;
		}
	}

	return;
}

//===========================================================================//

std::vector<int> pivotCustom(MatXf &J, bool pivot)
{
#if 1
	MatXf Jpivot = J;

	std::vector<int> indices;
	for(int i = 0; i < J.rows(); ++i)
		indices.push_back(i);

	for(int i = 0; i < J.cols(); ++i)
	{
		int j = i;

		while(j < J.rows())
		{
			if(fabs(Jpivot(j,i)) > 0.01f)
				break;
			else
				j++;
		}

		if(j == J.rows())
			j--;

		std::swap(indices[i], indices[j]);
		Jpivot.row(i).swap(Jpivot.row(j));
	}

	if(pivot)
		J = Jpivot;

	return indices;
#elif 0
	MatXf Jpivot = J;
	//std::vector<int> indices = farthestPoints(Jpivot, true); // angle
	std::vector<int> indices = farthestPoints(Jpivot, false); // distance
	if(pivot) J = Jpivot;
	return indices;
#else
	std::vector<int> indices;
	maxvol(J, indices);
	return indices;
#endif
}

//===========================================================================//

std::vector<int> farthestPoints(MatXf &rvs, bool a)
{
	size_t N = rvs.rows();
	size_t k = 0;

	size_t imax;
	float  dmax, d;

	std::vector<float> dist(N, std::numeric_limits<float>::max());
	std::vector<int>   idxs(N);

	for(size_t i = 0; i < N; ++i)
		idxs[i] = i;

	while(true)
	{
		RowVectorXf v = rvs.row(k);

		if(++k == N)
			break;

		imax = k;
		dmax = 0.0f;

		for(size_t i = k; i < N; ++i)
		{
			RowVectorXf w = rvs.row(i);

			if(a) // angle in rad
				//d = acos( v.dot(w) / ( v.norm() * w.norm() ) );
				d = -fabs( v.dot(w) / ( v.norm() * w.norm() ) );
			else // squared euclidean distance
				d = (w-v).squaredNorm();

			if(d < dist[i])
				dist[i] = d;

			if(dist[i] > dmax)
			{
				dmax = dist[i];
				imax = i;
			}
		}

		rvs.row(k).swap(rvs.row(imax));
		std::swap(idxs[k], idxs[imax]);
		std::swap(dist[k], dist[imax]);
	}

#if 1
	{
		using namespace std;
		cout << "shuffled indices: ";
		for(size_t i = 0; i < idxs.size(); ++i)
			cout << idxs[i] << " ";
		cout << endl;
		cout << "maximal distances: ";
		for(size_t i = 0; i < dist.size(); ++i)
			cout << dist[i] << " ";
		cout << endl;
	}
#endif

	return idxs;
}

//===========================================================================//

float fpo_mindist(
	const RowVectorXf &y,
	const std::vector<RowVectorXf> &Ys,
	size_t i, bool angleMetric)
{
#if 1
	float dy = std::numeric_limits<float>::max();

	for(size_t j = 0; j < Ys.size(); ++j)
	{
		if(j == i)
			continue;

		float d;

		if(angleMetric)
			//d = acos( y.dot(Ys[j]) / ( y.norm() * Ys[j].norm() ) );
			d = -fabs( y.dot(Ys[j]) / ( y.norm() * Ys[j].norm() ) );
			//d = -fabs( y.dot(Ys[j]) );
		else
			//d = (y.normalized() - Ys[j].normalized()).norm();
			d = (y - Ys[j]).norm();

		if(d < dy)
		{
			dy = d;
		}
	}

	return dy;
#else
	size_t N = Ys.size();
	size_t n = Ys.size();
	size_t d = y.cols();

	if(i == N) // y not in Ys
		n++;

	MatXf A(n,d);

	for(size_t i = 0; i < N; ++i)
		A.row(i) = Ys[i];

	if(i == N) // y not in Ys
		A.row(N) = y;

	MatXf AAT = A * A.transpose();

	return sqrt(AAT.determinant());
#endif
}

std::vector<int> fpoSubset(MatXf &rvs, int num, bool a)
{
	// farthest point subset optimization similar to Schloemer et al.

	std::vector<RowVectorXf> Xs, Ys; // samples
	std::vector<int>         Xi, Yi; // indices

	int rows = rvs.rows();

	if(num >= rows || num < 1)
	{
		for(int i = 0; i < rows; ++i)
			Yi.push_back(i);

		return Yi;
	}

	// pick num random rows of rvs and place them in Y, others in X

	for(int i = 0; i < rows; ++i)
	{
		if(i < num)
		{
			Ys.push_back(rvs.row(i));
			Yi.push_back(i);
		}
		else
		{
			Xs.push_back(rvs.row(i));
			Xi.push_back(i);
		}
	}

#if 0
	// normalize all rows
	for(int i = 0; i < Ys.rows(); ++i)
		if(Ys.row(i).norm() > 0)
			Ys.row(i).normalize();
	for(int i = 0; i < Xs.rows(); ++i)
		if(Xs.row(i).norm() > 0)
			Xs.row(i).normalize();
#endif


	// repeat until Y does not change
	bool changed = true;

	while(changed)
	{
		changed = false;

		// for each row y in Y
		for(int i = 0; i < num; ++i)
		{
			// calculate mindist dy of y to any point in Y
			RowVectorXf y = Ys[i];
			size_t iy = Yi[i];
			float dy = fpo_mindist(y, Ys, i, a);

			// select (f, df) = (y, dy)
			RowVectorXf  f = y;
			float       df = dy;
			size_t      fi = Yi[i];

			// remove y from Y and insert it into X
			std::vector<RowVectorXf> Ytmp = Ys;
			std::vector<RowVectorXf> Xtmp = Xs;

			std::vector<int> Yidx = Yi;
			std::vector<int> Xidx = Xi;

			Xtmp.push_back(y);
			Xidx.push_back(fi);

			Ytmp.erase(Ytmp.begin() + i);
			Yidx.erase(Yidx.begin() + i);

			// for each row x in X
			size_t ix = Xtmp.size()-1;
			for(size_t j = 0; j < Xtmp.size(); ++j)
			{
				// calculate mindist dx of x to any point in Y
				RowVectorXf x = Xtmp[j];
				float dx = fpo_mindist(x, Ytmp, Ytmp.size(), a);

				// if dx > df select (f, df) = (x, dx)
				if(dx > df)
				{
					f  = x;
					df = dx;
					fi = Xidx[j];
					ix = j;
				}
			}

			// check if Y changed
			if(fi != iy)
				changed = true;

			// remove f from X and insert into Y
			Ytmp.insert(Ytmp.begin() + i, f);
			Yidx.insert(Yidx.begin() + i, fi);

			Xtmp.erase(Xtmp.begin() + ix);
			Xidx.erase(Xidx.begin() + ix);

			Xs = Xtmp;
			Ys = Ytmp;

			Xi = Xidx;
			Yi = Yidx;
		}
	}

	// return Y as the fbo subset of rvs
	return Yi;
}

//===========================================================================//

/*
 * interpolate n values between t2, t1 and n1, n2:
 *
 *   t2  t1  t+0  t+1  ...  t+n  n1  n2
 *
 * subject to the known intermediate known values ks at the known indices is:
 *
 *   t2  t1  t+0  ...  ks[is[0]]  ...  ks[is[k]]  ... t+n  n1  n2
 */
std::vector<float> interpolate(
        size_t n,
        float t2, float t1,
        float n1, float n2,
        const std::vector<float> &ks,
        const std::vector<size_t> &is)
{
    size_t k = ks.size();
    size_t rows = n+2+4+k;
    size_t cols = n+4;
    float lambda = 100.0f;


    // lhs

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(rows, cols);

    for(size_t i = 0; i < n+2; ++i)
    {
        A(i, i+0) =  1.0f;
        A(i, i+1) = -2.0f;
        A(i, i+2) =  1.0f;
    }

    A(n+2,     0) = lambda;
    A(n+3,     1) = lambda;
    A(n+4, n+4-2) = lambda;
    A(n+5, n+4-1) = lambda;

    for(size_t i = 0; i < ks.size(); ++i)
    {
        size_t index = is[i];
        A(n+2+4+i, index+2) = lambda;
    }


    // rhs

    Eigen::VectorXf b = Eigen::VectorXf::Zero(rows);

    b(n+2) = lambda * t2;
    b(n+3) = lambda * t1;
    b(n+4) = lambda * n1;
    b(n+5) = lambda * n2;

    for(size_t i = 0; i < k; ++i)
    {
        size_t index = n+2+4+i;
        b(index) = lambda * ks[i];
    }


    // solve ATAx=ATb

    Eigen::LDLT<Eigen::MatrixXf> solver(A.transpose() * A);
    Eigen::VectorXf x = solver.solve(A.transpose() * b);

    std::vector<float> interpolated_values(x.rows());
    for(int i = 0; i < x.rows(); ++i)
        interpolated_values[i] = x(i);

    return interpolated_values;
}

/*

// example usage of interpolate():

int main(int argc, char **argv)
{
    // no data available between frames t and t+n

    size_t n = 10; // interpolate n values

    float t2 = 0.1f; // known value at t-2
    float t1 = 0.2f; // known value at t-1

    float n1 = 1.0f; // known value at t+n+1
    float n2 = 0.9f; // known value at t+n+2

    std::vector<float> known_values; // intermediate known values
    std::vector<size_t> known_indices; // intermediate known indices

    // value 2.0 at index t+5
    known_indices.push_back(5);
    known_values.push_back(2.0f);

    std::vector<float> interpolated_values = interpolate(n, t2, t1, n1, n2, known_values, known_indices);

    for(size_t i = 0; i < interpolated_values.size(); ++i)
        std::cout << interpolated_values[i] << std::endl;

    return 0;
}

*/

//===========================================================================//

} // namespace ik

