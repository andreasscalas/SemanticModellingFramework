#include "PCA.h"

#include <iostream>

//#define TIMER_STUFF
#ifdef TIMER_STUFF
#include <ICLUtils/StackTimer.h>
#endif

using namespace std;
using namespace Eigen;

namespace ik {

//===========================================================================//

PCA::PCA()
{
	numParameters = 0;
	numFixedDof = 0;
	numDof = 0;

	sMin = 0.1f;
	sMax = 2.0f;

	numCor = 10;
	bufferMax = 500;
	iterEM = 2;

	// timer stuff
	time_number = 0;
	time_total_sum = 0;
	time_construct_sum = 0;
	time_em_sum = 0;
	time_ortho_sum = 0;
	time_cov_sum = 0;
	time_decomp_sum = 0;
	time_evmat_sum = 0;
	time_init_sum = 0;
	time_update_sum = 0;
	time_reset = true;
	inc = true;
}

//===========================================================================//

PCA::PCA(
	const vector<vector<float> > &data,
	size_t dof, size_t fDof, size_t fDofData)
{
	// initialize variables

	numParameters = 0;
	numFixedDof = 0;
	numDof = 0;

	sMin = 0.1f;
	sMax = 2.0f;

	numCor = 10;
	bufferMax = 500;
	iterEM = 2;

	// timer stuff
	time_number = 0;
	time_total_sum = 0;
	time_construct_sum = 0;
	time_em_sum = 0;
	time_ortho_sum = 0;
	time_cov_sum = 0;
	time_decomp_sum = 0;
	time_evmat_sum = 0;
	time_init_sum = 0;
	time_update_sum = 0;
	time_reset = true;
	inc = true;

	// perform pca on data

	size_t num = data.size();
	if(!num)
		return;

	size_t dim = data[0].size() - fDofData;
	if(!dof)
		dof = dim;

	numParameters = fDof + dim;
	numFixedDof = fDof;
	numDof = dof;

	// construct data matrix from input vector
	MatXf D(num, dim);
	for(size_t i = 0; i < num; ++i)
		for(size_t j = 0; j < dim; ++j)
			D(i, j) = data[i][fDofData + j];

#ifdef TIMER_STUFF
icl::utils::Time t = icl::utils::Time::now();
#endif

	// compute mean-centered covariance matrix
	RowVectorXf M = D.colwise().mean();
	D.rowwise() -= M;
	MatXf C = D.adjoint() * D / num;

#ifdef TIMER_STUFF
float ms_cov = t.age().toMicroSeconds();
t = icl::utils::Time::now();
#endif

	// perform eigenvalue decomposition
	SelfAdjointEigenSolver<MatXf> E(C);

#ifdef TIMER_STUFF
float ms_eig = t.age().toMicroSeconds();
std::cout << "covariance construction:  " << ms_cov << "us" <<std::endl;
std::cout << "eigenvalue decomposition: " << ms_eig << "us" <<std::endl;
std::cout << "EIGENVALUES: " << endl << E.eigenvalues() << endl;
std::cout << "EIGENVECTORS: " << endl << E.eigenvectors() << endl;
#endif


#if 1
	// comparison with SVD on mean-centered D using Eigen::JacobiSVD
    //t = icl::utils::Time::now();

	Eigen::JacobiSVD<MatXf> svd(D/sqrt(num), ComputeThinV);

    //float ms_svd = t.age().toMicroSeconds();

	std::cout << "[SVD] singular values:" << endl << svd.singularValues() << endl;
	std::cout << "[SVD] singular vectors:" << endl << svd.matrixV() << endl;
    //std::cout << "[SVD] computation time: " << ms_svd << "us" << std::endl;

	MatXf svdV=svd.matrixV();
	cout<<svdV.col(0).normalized()<<endl;
	cout<<"--"<<endl;
	cout<<E.eigenvectors().col(19)<<endl;
#endif


	// rearrange eigenvectors
	MatXf evs = E.eigenvectors();
	for(int i = 0; i < evs.cols() / 2; ++i)
		evs.col(i).swap(evs.col(evs.cols() - i - 1));

	// construct principal component matrix pcs
	pcs = MatXf::Zero(fDof + dim, fDof + dof);
	pcs.block(0, 0, fDof, fDof) << MatXf::Identity(fDof, fDof);
	pcs.block(fDof, fDof, dim, dof) << evs.block(0, 0, dim, dof);

	// store anchor matrix
	as = pcs;
	aaT = as * as.transpose();

	// construct eigenvalue vector eig
	eig = vector<float>(dim, 0);
	for(size_t i = 0; i < dim; ++i)
		eig[i] = E.eigenvalues()((dim - 1) - i);

	// construct data mean vector mid
	mid = vector<float>(fDof + dim, 0);
	for(size_t i = 0; i < fDof; ++i)
		mid[i] = 0;
	for(size_t i = fDof; i < fDof + dim; ++i)
		mid[i] = M(i - fDof);

#if 0
	cout << "====================================================" << endl;
	MatXf pcamat = pcs.block(6,6,pcs.rows()-6,pcs.cols()-6);
	cout << "PCA matrix:" << endl << pcamat << endl<<endl;
	cout << "PCA matrix [abs]:" << endl << pcamat.cwiseAbs() << endl<<endl;
	cout << "Eigenvalues: " << endl << E.eigenvalues() << endl<<endl;
	cout << "PCA matrix row sums: " << endl << pcamat.cwiseAbs().rowwise().sum() << endl<<endl;
	MatXf evwrs = MatXf::Zero(pcamat.rows(), 1);
	for(int i = 0; i < pcamat.rows(); ++i)
		for(int j = 0; j < pcamat.cols(); ++j)
			evwrs(i,0) += fabs(pcamat(i,j)) * fabs(eig[j]);
	cout << "Eigenvalue-weighted row sums: " << endl << evwrs << endl << endl;
	cout << "====================================================" << endl;
#endif

#if 0
	pcs = MatXf::Identity(26, 6 + 1);
	pcs.block(6, 6, 20, 1) = MatXf::Ones(20, 1) * 1.0f / 20.0f;
	as = pcs;
	aaT = as * as.transpose();
	eig = vector<float>(20, 1.0f / 20.0f);
	mid = vector<float>(26, 0);
#endif
}

//===========================================================================//

std::vector<float> PCA::toPC(const vector<float> &param, size_t newDof) const
{
	if(!pcs.size())
		return param;

	size_t dim = numParameters;
	size_t dof = numFixedDof + (newDof ? newDof : numDof);

	VecXf p(dim);
	for(size_t i = 0; i < dim; ++i)
		p[i] = param[i] - mid[i];

	p = pcs.transpose() * p;

	vector<float> result(dof, 0);
	for(size_t i = 0; i < dof; ++i)
		result[i] = p[i];

	return result;
}

//===========================================================================//

std::vector<float> PCA::fromPC(const vector<float> &param, size_t newDof) const
{
	if(!pcs.size())
		return param;

	size_t dim = numParameters;
	size_t dof = numFixedDof + numDof;

	VecXf p(dof);

	for(size_t i = 0; i < dof; ++i)
	{
		if(newDof && i >= numFixedDof + newDof)
			p[i] = 0;
		else
			p[i] = param[i];
	}

	p = pcs * p;

	vector<float> result(dim, 0);
	for(size_t i = 0; i < dim; ++i)
		result[i] = p[i] + mid[i];

	return result;
}

//===========================================================================//

MatXf PCA::getPCs(size_t dof) const
{
	if(!pcs.size())
		return MatXf::Identity(numParameters, numParameters);

	if(dof && dof < (size_t)numDof)
		return MatXf(pcs.block(0, 0, numParameters, numFixedDof + dof));

	return pcs;
}

//===========================================================================//

VecXf PCA::adapt(
	const vector<float> &param)
{
	if(!as.size())
		return VecXf();

	// compute projected residual sample
	VecXf p(numParameters);
	VecXf m(numParameters);
	for(int i = 0; i < numParameters; ++i)
	{
		p[i] = param[i];
		m[i] = mid[i];
	}
	VecXf sample = (p - m) - aaT * (p - m);

	// check validity of sample
	if(isValid(sample))
	{
		// in-place covariance computation
		bool inPlaceCov = true;
		if(inPlaceCov)
		{
			// update sample buffer
			VecXf sample_out = VecXf::Zero(sample.size());
			buffer.push_back(sample);
			if(buffer.size() > (size_t)bufferMax)
			{
				sample_out = buffer.front();
				buffer.pop_front();
			}

			int d = numParameters - numFixedDof;

			// not enough samples
			if(buffer.size() < (size_t)d)
				return VecXf();

#ifdef TIMER_STUFF
	icl::utils::Time t_total = icl::utils::Time::now();
	icl::utils::Time t_init = icl::utils::Time::now();
#endif

			// initialize
			if(!cs.size() || buffer.size() < (size_t)bufferMax)
			{
				// mean
				s_mean = VecXf::Zero(d);
				for(size_t i = 0; i < buffer.size(); ++i)
				{
					VecXf s_i = buffer[i].block(numFixedDof,0,d,1);
					s_mean += s_i;
				}
				s_mean /= buffer.size();

				// covariance matrix
				C_ = MatXf::Zero(d,d);
				for(size_t i = 0; i < buffer.size(); ++i)
				{
					VecXf s_i = buffer[i].block(numFixedDof,0,d,1);
					C_ += (s_i - s_mean) * (s_i - s_mean).transpose();
				}
				MatXf C=C_;

				/*
				// homogenous covariance
				M_ = MatXf::Zero(d+1,d+1);
				for(size_t i = 0; i < buffer.size(); ++i)
				{
					VecXf s_i_h = VecXf::Ones(d+1);
					s_i_h.block(0,0,d,1) = buffer[i].block(numFixedDof,0,d,1);
					M_ += s_i_h * s_i_h.transpose();
				}

				// mean center
				VecXf t = -s_mean;
				MatXf M = M_;
				for(int j = 0; j < d+1; ++j)
					for(int i = 0; i < d; ++i)
						M(i,j) += t[i] * M(d,j);
				for(int i = 0; i < d+1; ++i)
					for(int j = 0; j < d; ++j)
						M(i,j) += t[j] * M(i,d);

				//std::cout << "covariance difference:" << std::endl << (M.block(0,0,d,d) - C).norm() << std::endl;

				MatXf C = M.block(0,0,d,d);
				*/

				// perform eigenvalue decomposition
				SelfAdjointEigenSolver<MatXf> E(C);

				// rearrange eigenvectors
				MatXf evs = E.eigenvectors();
				for(int i = 0; i < evs.cols() / 2; ++i)
					evs.col(i).swap(evs.col(evs.cols() - i - 1));

				// construct corrective principal component matrix cs
				cs = MatXf::Zero(numParameters, numCor);
				cs.block(numFixedDof, 0, numParameters - numFixedDof, numCor)
					<< evs.block(0, 0, numParameters - numFixedDof, numCor);

				// update anchors in pc-matrix, increment dofs
				if(inc)
				{
					pcs = MatXf(as.rows(), as.cols() + cs.cols());
					pcs.block(0,0,as.rows(),as.cols()) = as;

					numDof += numCor;
					inc = false;
				}

				// update correctives in pc-matrix
				pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;
			}

#ifdef TIMER_STUFF
	float time_init = t_init.age().toMicroSeconds();
	icl::utils::Time t_update = icl::utils::Time::now();

	float time_update_1;
	float time_update_2;
#endif

			// what is the minimum buffer size before the update?
			// -> sammeln bis bufferMax, dann einmal init, dann immer update

			// update
			if(buffer.size() == (size_t)bufferMax)
			{
#ifdef TIMER_STUFF
	icl::utils::Time t_update_1 = icl::utils::Time::now();
#endif

				VecXf s_in = sample.block(numFixedDof,0,d,1);
				VecXf s_out = sample_out.block(numFixedDof,0,d,1);

				VecXf s_mean_prev = s_mean;
				float N = (float)buffer.size();
				s_mean += s_in / N - s_out / N;

				for(int i = 0; i < d; ++i)
					for(int j = 0; j < d; ++j)
						C_(i,j) += s_mean_prev[i] * s_mean_prev[j]
								 - s_mean[i] * s_mean[j]
								 - s_out[i] * s_out[j] / N
								 + s_in[i] * s_in[j] / N;
				MatXf C=C_;

				/*
				// homogenous update
				VecXf u = s_in - s_out;
				VecXf u_h = VecXf::Ones(d+1);
				u_h.block(0,0,d,1) = u;

				s_mean += u / buffer.size();

				//for(int i = 0; i < d+1; ++i)
					//for(int j = 0; j < d+1; ++j)
						//M_(i,j) += u_h[i] * u_h[j]; // wrong

				for(int i = 0; i < d; ++i)
					for(int j = 0; j < d; ++j)
						M_(i,j) += (s_in[i] * s_in[j]) - (s_out[i] * s_out[j]);

				VecXf t = -s_mean;
				MatXf M = M_;
				for(int j = 0; j < d+1; ++j)
					for(int i = 0; i < d; ++i)
						M(i,j) += t[i] * M(d,j);
				for(int i = 0; i < d+1; ++i)
					for(int j = 0; j < d; ++j)
						M(i,j) += t[j] * M(i,d);

				MatXf C = M.block(0,0,d,d);
				*/

#ifdef TIMER_STUFF
	time_update_1 = t_update_1.age().toMicroSeconds();
	icl::utils::Time t_update_2 = icl::utils::Time::now();
#endif

				// perform eigenvalue decomposition
				SelfAdjointEigenSolver<MatXf> E(C);

				// rearrange eigenvectors
				MatXf evs = E.eigenvectors();
				for(int i = 0; i < evs.cols() / 2; ++i)
					evs.col(i).swap(evs.col(evs.cols() - i - 1));

				// construct corrective principal component matrix cs
				cs = MatXf::Zero(numParameters, numCor);
				cs.block(numFixedDof, 0, numParameters - numFixedDof, numCor)
					<< evs.block(0, 0, numParameters - numFixedDof, numCor);

				// update correctives in pc-matrix
				pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;

#ifdef TIMER_STUFF
	time_update_2 = t_update_2.age().toMicroSeconds();
#endif
			}

#ifdef TIMER_STUFF
	float time_update = t_update.age().toMicroSeconds();
	float time_total = t_total.age().toMicroSeconds();

	time_number 		+= 1.0f;
	time_total_sum 		+= time_total;
	time_init_sum 		+= time_init;
	time_update_sum		+= time_update;

	std::cout << "---" << std::endl << std::endl;
	std::cout << "standard pca [in-place covariance]" << std::endl << std::endl;

	std::cout << "total apca update: "
			  << time_total_sum / time_number << " us [" << time_total << "]" << std::endl;
	std::cout << "in-place init:     "
			  << time_init_sum / time_number << " us [" << time_init << "]" << std::endl;
	std::cout << "in-place update:   "
			  << time_update_sum / time_number << " us [" << time_update << "]" << std::endl;
	std::cout << "update time 1: " << time_update_1 << std::endl;
	std::cout << "update time 2: " << time_update_2 << std::endl;
	std::cout << std::endl;

	std::cout << "BUFFER MAX:  " << bufferMax << std::endl;
	std::cout << "BUFFER SIZE: " << buffer.size() << std::endl;
	std::cout << std::endl;

	if(time_reset && buffer.size() == (size_t)bufferMax)
	{
		time_number = 0;
		time_total_sum = 0;
		time_construct_sum = 0;
		time_em_sum = 0;
		time_ortho_sum = 0;
		time_cov_sum = 0;
		time_decomp_sum = 0;
		time_evmat_sum = 0;
		time_init_sum = 0;
		time_update_sum = 0;
		time_reset = false;
	}
#endif

			return sample;
		}


		//-- in-place coviariace end --


		// update sample buffer
		buffer.push_back(sample);
		if(buffer.size() > (size_t)bufferMax)
			buffer.pop_front();

		// initialize correctives if necessary/possible
		if(!cs.size() && buffer.size() >= (size_t)numCor)
		{
			// construct correctives from buffer without pose dofs
			cs = MatXf(numParameters - numFixedDof, numCor);
			for(int i = 0; i < numParameters - numFixedDof; ++i)
				for(int j = 0; j < numCor; ++j)
					cs(i,j) = buffer[j][numFixedDof + i];

			// orthogonalize matrix wit QR decomposition
			ColPivHouseholderQR<MatXf> qr(cs);
			cs = MatXf(qr.matrixQ()).block(
					0,0, numParameters - numFixedDof, numCor);

			// add pose dofs to corrective matrix
			MatXf tmp = cs;
			cs = MatXf(numParameters, numCor);
			for(int i = 0; i < numParameters; ++i)
			{
				for(int j = 0; j < numCor; ++j)
				{
					if(i < numFixedDof)
						cs(i,j) = 0;
					else
						cs(i,j) = tmp(i - numFixedDof, j);
				}
			}

			// join anchors and correctives in pc-matrix
			pcs = MatXf(as.rows(), as.cols() + cs.cols());
			pcs.block(0,0,as.rows(),as.cols()) = as;
			pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;

			numDof += numCor;
		}
		// update correctives if possible
		else if(cs.size())
		{
#if 0

#ifdef TIMER_STUFF
	icl::utils::Time t_total = icl::utils::Time::now();
	icl::utils::Time t_construct = icl::utils::Time::now();
#endif

			typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatXf2;

			// construct buffer matrix S
			//MatXf2 S(numParameters, buffer.size());
			//for(size_t j = 0; j < buffer.size(); ++j)
				//for(int i = 0; i < numParameters; ++i)
					//S(i,j) = buffer[j][i];

			MatXf2 S(numParameters - numFixedDof, buffer.size());
			for(size_t j = 0; j < buffer.size(); ++j)
				for(int i = 0; i < numParameters - numFixedDof; ++i)
					S(i,j) = buffer[j][i + numFixedDof];

#ifdef TIMER_STUFF
	float time_construct = t_construct.age().toMicroSeconds();
	icl::utils::Time t_em = icl::utils::Time::now();
#endif

			// iterative expectation maximization
			MatXf ys;
			MatXf xs = cs.block(numFixedDof, 0, cs.rows() - numFixedDof, cs.cols());
			for(int i = 0; i < iterEM; ++i)
			{
				//ys = ( cs.transpose() * cs ).inverse() * cs.transpose() * S;
				//cs = S * ys.transpose() * ( ys * ys.transpose() ).inverse();

				ys = ( xs.transpose() * xs ).inverse() * xs.transpose() * S;
				xs = S * ys.transpose() * ( ys * ys.transpose() ).inverse();
			}

#ifdef TIMER_STUFF
	float time_em = t_em.age().toMicroSeconds();
	icl::utils::Time t_ortho = icl::utils::Time::now();
#endif

			// orthogonalize result with QR decomposition
			//MatXf tmp = cs.block(numFixedDof, 0, cs.rows() - numFixedDof, cs.cols());
			//ColPivHouseholderQR<MatXf> qr(tmp);
			//cs.block(numFixedDof, 0, cs.rows() - numFixedDof, cs.cols())
				//= MatXf(qr.matrixQ()).block(0, 0, numParameters - numFixedDof, numCor);

			ColPivHouseholderQR<MatXf> qr(xs);
			cs.block(numFixedDof, 0, cs.rows() - numFixedDof, cs.cols())
				= MatXf(qr.matrixQ()).block(0, 0, numParameters - numFixedDof, numCor);

#ifdef TIMER_STUFF
	float time_ortho = t_ortho.age().toMicroSeconds();
	float time_total = t_total.age().toMicroSeconds();

	time_number 		+= 1.0f;
	time_total_sum 		+= time_total;
	time_construct_sum 	+= time_construct;
	time_em_sum 		+= time_em;
	time_ortho_sum 		+= time_ortho;

	std::cout << "---" << std::endl << std::endl;
	std::cout << "em pca" << std::endl << std::endl;

	std::cout << "total apca update:        "
			  << time_total_sum / time_number << " us [" << time_total << "]" << std::endl;
	std::cout << "data matrix construction: "
			  << time_construct_sum / time_number << " us [" << time_construct << "]" << std::endl;
	std::cout << "expectation maximization: "
			  << time_em_sum / time_number << " us [" << time_em << "]" << std::endl;
	std::cout << "matrix orthogonalization: "
			  << time_ortho_sum / time_number << " us [" << time_ortho << "]" << std::endl;
	std::cout << std::endl;

	std::cout << "BUFFER MAX:  " << bufferMax << std::endl;
	std::cout << "BUFFER SIZE: " << buffer.size() << std::endl;
	std::cout << std::endl;

	if(time_reset && buffer.size() == (size_t)bufferMax)
	{
		time_number = 0;
		time_total_sum = 0;
		time_construct_sum = 0;
		time_em_sum = 0;
		time_ortho_sum = 0;
		time_cov_sum = 0;
		time_decomp_sum = 0;
		time_evmat_sum = 0;
		time_init_sum = 0;
		time_update_sum = 0;
		time_reset = false;
	}
#endif // TIMER_STUFF

#else

//	icl::utils::Time t_total = icl::utils::Time::now();
//	icl::utils::Time t_construct = icl::utils::Time::now();

			// construct buffer matrix S
			MatXf S(buffer.size(), numParameters - numFixedDof);
			for(size_t j = 0; j < buffer.size(); ++j)
				for(int i = 0; i < numParameters - numFixedDof; ++i)
					S(j,i) = buffer[j][i + numFixedDof];

//	float time_construct = t_construct.age().toMicroSeconds();
//	icl::utils::Time t_cov = icl::utils::Time::now();

			// compute mean-centered covariance matrix
			//RowVectorXf M = S.colwise().mean();
			//S.rowwise() -= M;
			//MatXf C = S.adjoint() * S / buffer.size();
			;
			S/=sqrt((float)buffer.size());

//	float time_cov = t_cov.age().toMicroSeconds();
//	icl::utils::Time t_decomp = icl::utils::Time::now();

			// perform eigenvalue decomposition
			//SelfAdjointEigenSolver<MatXf> E(C);
			;
			Eigen::JacobiSVD<MatXf> svd(S, ComputeThinV);

//	float time_decomp = t_decomp.age().toMicroSeconds();
//	icl::utils::Time t_evmat = icl::utils::Time::now();

			// rearrange eigenvectors
			//MatXf evs = E.eigenvectors();
			//for(int i = 0; i < evs.cols() / 2; ++i)
				//evs.col(i).swap(evs.col(evs.cols() - i - 1));

			// construct corrective principal component matrix cs
			//cs = MatXf::Zero(numParameters, numCor);
			//cs.block(numFixedDof, 0, numParameters - numFixedDof, numCor)
				//<< evs.block(0, 0, numParameters - numFixedDof, numCor);
			;
			cs = MatXf::Zero(numParameters, numCor);
			cs.block(numFixedDof, 0, numParameters-numFixedDof, numCor)
				<< svd.matrixV().block(0, 0, numParameters-numFixedDof, numCor) * -1.0f;

//	float time_evmat = t_evmat.age().toMicroSeconds();
//	float time_total = t_total.age().toMicroSeconds();
/*
	time_number 		+= 1.0f;
	time_total_sum 		+= time_total;
	time_construct_sum 	+= time_construct;
	time_cov_sum 		+= time_cov;
	time_decomp_sum 	+= time_decomp;
	time_evmat_sum 		+= time_evmat;
*/

	std::cout << "---" << std::endl << std::endl;
	std::cout << "standard pca" << std::endl << std::endl;
/*
	std::cout << "total apca update:        "
			  << time_total_sum / time_number << " us [" << time_total << "]" << std::endl;
	std::cout << "data matrix construction: "
			  << time_construct_sum / time_number << " us [" << time_construct << "]" << std::endl;
	std::cout << "covariance computation:   "
			  << time_cov_sum / time_number << " us [" << time_cov << "]" << std::endl;
	std::cout << "eigenvalue decomposition: "
			  << time_decomp_sum / time_number << " us [" << time_decomp << "]" << std::endl;
	std::cout << "pc matrix construction:   "
			  << time_evmat_sum / time_number << " us [" << time_evmat << "]" << std::endl;
	std::cout << std::endl;
*/
	std::cout << "BUFFER MAX:  " << bufferMax << std::endl;
	std::cout << "BUFFER SIZE: " << buffer.size() << std::endl;
	std::cout << std::endl;

	if(time_reset && buffer.size() == (size_t)bufferMax)
	{
		time_number = 0;
		time_total_sum = 0;
		time_construct_sum = 0;
		time_em_sum = 0;
		time_ortho_sum = 0;
		time_cov_sum = 0;
		time_decomp_sum = 0;
		time_evmat_sum = 0;
		time_init_sum = 0;
		time_update_sum = 0;
		time_reset = false;
	}

#endif

			// update correctives in pc-matrix
			pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;
		}
	}

	return sample;
}

//===========================================================================//

void PCA::reset()
{
	if(!pcs.size())
		return;

	pcs = as;
	cs = MatXf();
	buffer.clear();
	numDof = as.cols() - numFixedDof;
}

//===========================================================================//

void PCA::expand()
{
	if(!pcs.size())
		return;

	as = pcs;
	aaT = as * as.transpose();
	cs = MatXf();
	buffer.clear();
	numDof = as.cols() - numFixedDof;
}

//===========================================================================//

std::vector<float> PCA::getAdaptiveEigenvalues()
{
	return eig;
	// adaptive eigenvalues:
	// - construct buffer data matrix (mean center?)
	// - project buffer matrix into corrective space
	// - calculate covariance matrix of projected data
	// - perform eigenvalue decomposition on covariance
	// - return joint anchor and corretive eigenvalues
}
			
//===========================================================================//

} // namespace ik

