#ifndef MAPPING_H
#define MAPPING_H

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <limits>

#include "../math/Math.h"
#include "../math/PCA.h"

namespace ik {

//===========================================================================//

	enum AxisType
	{
		ROTATION_AXIS,
		TRANSLATION_AXIS,
		COUPLED_TRANSLATION_AXIS
	};

//===========================================================================//

	struct AxisInfo
	{
		Vec3f axis;
		int index;
		AxisType type;

		AxisInfo(const Vec3f &a, int i, AxisType t)
			: axis(a), index(i), type(t) {}

		AxisInfo() {}
	};

//===========================================================================//

	struct JointInfo
	{
		std::string joint;
		Vec3f axis;
		AxisType type;

		JointInfo(const std::string &j, const Vec3f &a, AxisType t)
			: joint(j), axis(a), type(t) {}

		JointInfo() {}
	};

//===========================================================================//

	class Mapping
	{
		public:

			Mapping(int num = 0);

			void init();

			void map(
					int index,
					const std::string &joint,
					const Vec3f &axis,
					AxisType type = ROTATION_AXIS);

			void map(
					int index,
					const std::string &joint,
					const Vec3f &axis,
					AxisType type,
					float min,
					float max);

			void withPose(
					const std::string &joint = "pose",
					int r1 = 0, int r2 = 1, int r3 = 2);

			void withScale(
					const std::string &joint = "scale");

			void setDefaults(bool definePose = true, bool defineForearm = false);
			void setDefaultsR(bool definePose = true, bool defineForearm = false);

			bool hasPoseJoint();
			bool hasScaleJoint();

			std::string getPoseJoint();
			std::string getScaleJoint();

			void setPoseJoint(const std::string &name) {
				poseJoint = name;
			}
			void setScaleJoint(const std::string &name) {
				scaleJoint = name;
			}

			Vec3f getRotationAxis(int i);
			Eigen::Vector3i getRotationAxes();

			Vec3f toEulerAngles(const Mat3f &m);
			Vec3f toEulerAngles(const Mat4f &m);
			Mat3f fromEulerAngles(const Vec3f &v);
			Mat3f fromEulerAngles(float a, float b, float c);

		public:

            std::vector<float> constrain(const std::vector<float> &param) const;
			float constrain(int index, float current, float delta);

            float getMin(int index) const;
            float getMax(int index) const;

            bool hasLimits(int index) const {
				return (
					mins.find(index) != mins.end() &&
					maxs.find(index) != maxs.end() );
			}

			std::vector<float> constrainPC(const std::vector<float> &param);
			float constrainPC(int index, float current, float delta);

            float getMinPC(int index) const;
            float getMaxPC(int index) const;

            bool hasLimitsPC(int index) const {
				return (
					minsPC.find(index) != minsPC.end() &&
					maxsPC.find(index) != maxsPC.end() );
			}

			void setMin(int index, float val) { mins[index] = val; }
			void setMax(int index, float val) { maxs[index] = val; }

			void setMinPC(int index, float val) { minsPC[index] = val; }
			void setMaxPC(int index, float val) { maxsPC[index] = val; }

		public:

			void doPCA(
					const std::vector<std::vector<float> > &data,
					bool setLimits = false, size_t dof = 0, size_t fixedDofData = 0);

			void adaptPCA(
					const std::vector<float> &posture);

			void resetPCA();

			void expandPCA();

            std::vector<float> toPC(const std::vector<float> &param, size_t newDof = 0) const;
            std::vector<float> fromPC(const std::vector<float> &param, size_t newDof = 0) const;

            MatXf getPCs(size_t dof = 0) const;
			std::vector<float> getEigenvalues();
			std::vector<float> getMidpoint();

			PCA &getPCA() { return pca; }

			void setParameterLimits(
					const std::vector<std::vector<float> > &data,
					size_t dof = 0, size_t fixedDof = 0);

            std::vector<AxisInfo> getAxisInfos(const std::string &joint) const;
            JointInfo getJointInfo(int index) const;

            int getNumberOfParameters() const;
            int getDof() const;
            int getFixedDof() const;
			void setFixedDof(int fDof);

			void setPCAEnabled(bool b);
            bool getPCAEnabled() const;

			int getDimension() {
				if(enablePCA)
					return numFixedDof + numDof;
				else
					return numParameters;
			}

		public:

			static Mapping leftHand() {
				Mapping m;
				m.setDefaults();
				return m;
			}

			static Mapping leftArm() {
				Mapping m;
				m.setDefaults(true, true);
				return m;
			}

			static Mapping rightHand() {
				Mapping m;
				m.setDefaultsR();
				return m;
			}

			static Mapping rightArm() {
				Mapping m;
				m.setDefaultsR(true, true);
				return m;
			}

		protected:

			std::map< std::string, std::vector<AxisInfo> > axisInfos;
			std::map< int, JointInfo > jointInfos;

			std::map<int, float> mins;
			std::map<int, float> maxs;

			std::map<int, float> minsPC;
			std::map<int, float> maxsPC;

			int count;
			int numParameters;
			int numFixedDof;
			int numDof;

			PCA pca;

			bool enablePCA;
			bool hasPose;
			bool hasScale;

			std::string poseJoint;
			std::string scaleJoint;

			Vec3f rotationAxis1;
			Vec3f rotationAxis2;
			Vec3f rotationAxis3;

			int e1, e2, e3;

	};

} // namespace ik

#endif

