#pragma once

#include <vector>
#include <deque>
#include "Math.h"

// pca usage:
// - inputs are database, desired pc space dimension and fixed dof.
// - fixed dof are the parts of the kinematic model that are unaffected by the
//   pc space mapping because they are not contained in the database. they must
//   be defined as the first elements of the joint angle / pc space vectors.
//   for me, these are the six global translation / rotation parameters.
// - use fromPC and toPC to convert between joint angle space and pc space.

namespace ik {

	class PCA
	{
		public:

			PCA();

			PCA(const std::vector<std::vector<float> > &data,
				size_t dof = 0, size_t fixedDof = 0, size_t fixedDofData = 0);

			std::vector<float> toPC(const std::vector<float> &param, size_t newDof = 0) const;
			std::vector<float> fromPC(const std::vector<float> &param, size_t newDof = 0) const;

			MatXf getPCs(size_t dof) const;

			VecXf adapt(const std::vector<float> &param);
			void reset();
			void expand();

		public:

			void setNumberOfCorrectives(int n) { numCor = n; }
			int getNumberOfCorrectives() { return numCor; }

			void setBufferSize(int bufMax) { bufferMax = bufMax; }
			int getBufferSize() { return bufferMax; }

			void setEMIterations(int itEM) { iterEM = itEM; }
			int getEMIterations() { return iterEM; }

			void setSampleMin(float min) { sMin = min; }
			float getSampleMin() { return sMin; }

			void setSampleMax(float max) { sMax = max; }
			float getSampleMax() { return sMax; }

			bool isValid(VecXf &sample) {
				float s = sample.norm();
				return (s > sMin && s < sMax);
			}

			std::vector<float> getAdaptiveEigenvalues();
			
		public:

			MatXf pcs;

			std::vector<float> eig;
			std::vector<float> mid;

			MatXf as;
			MatXf cs;
			MatXf aaT;

			std::deque<VecXf> buffer;

			int numParameters;
			int numFixedDof;
			int numDof;

			int numCor;
			int bufferMax;
			int iterEM;

			float sMin;
			float sMax;

			// in-place covariance
			VecXf s_mean;
			MatXf M_;
			MatXf C_;
			bool inc;

			// timer stuff
			float time_number;
			float time_total_sum;
			float time_construct_sum;
			float time_em_sum;
			float time_ortho_sum;
			float time_cov_sum;
			float time_decomp_sum;
			float time_evmat_sum;
			float time_init_sum;
			float time_update_sum;
			bool time_reset;
	};

} // namespace ik

