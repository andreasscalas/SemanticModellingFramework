#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "Skeleton.h"

// gmm test
//#include <math/ExpectationMaximization.h"
//#include <util/PostureFile.h"

namespace ik {

	class InverseKinematics
	{
		public:

			InverseKinematics(float lambda = 1.0);

			void solve(Skeleton &skeleton);

			float getLambda() const { return lambda; }
			void setLambda(float lambda_) { lambda = lambda_; }

			float getDof() const { return dof; }
			void setDof(float dof_) { dof = dof_; }

			bool doRegularization() const { return doRegul; }
			void setRegularization(bool doRegul_) { doRegul = doRegul_; }

			bool doAlternateInversion() const { return altInv; }
			void setAlternateInversion(bool altInv_) { altInv = altInv_; }

			bool doClamping() const { return clamp; }
			void setClamping(bool clamp_) { clamp = clamp_; }

			bool doPointPlaneMinimization() const { return pplane; }
			void setPointPlaneMinimization(bool pplane_) { pplane = pplane_; }

			bool doPrediction() const { return doPredict; }
			void setPrediction(bool doPredict_) { doPredict = doPredict_; }

			bool doKeepDof() const { return keepDof; }
			void setKeepDof(bool keepDof_) { keepDof = keepDof_; }

			bool doSingleDof() const { return singleDof; }
			void setSingleDof(bool singleDof_) { singleDof = singleDof_; }

			bool doScaling() const { return doScale; }
			void setScaling(bool doScale_) { doScale = doScale_; }

			float getError() const { return error; }
			MatXf getJacobian() { return jacobian; }

			void setPredictionUpdate(const std::vector<float> &dt) { dthetaPrev = dt; doPredict = true; }
			std::vector<float> &getPredictionUpdate() { return dthetaPrev; }

		public:

			// ik
			void setup(Skeleton &skeleton);
			void predict(Skeleton &skeleton);
			MatXf dampingMatrix(Skeleton &skeleton);
			std::vector<float> compute(Skeleton &skeleton);
			void regularize(Skeleton &skeleton, std::vector<float> &dt);

			// util
			std::vector<float> factorVec(
					const std::vector<float> &v,
                    float factor = 1.0);

		protected:

			float lambda;
			float error;
			float scaleVal;

			size_t dof;

			MatXf errors;
			MatXf jacobian;

			std::vector<float> dthetaPrev;

			bool doRegul;
			bool altInv;
			bool clamp;
			bool pplane;
			bool doPredict;
			bool keepDof;
			bool singleDof;
			bool doScale;

			std::vector<float> params;
			std::vector<float> hprev;
/*
			// gmm test
			GaussianMixture gmm;
			void initGMM(Skeleton &skeleton);
            */

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};

} // namespace ik

#endif


