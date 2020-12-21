#include "Mapping.h"

#include <iostream>

using namespace std;
using namespace Eigen;

namespace ik {

//===========================================================================//

Mapping::Mapping(int num)
{
	count = 0;
	numParameters = num;
	numFixedDof = 0;
	numDof = 0;

	enablePCA = false;
	hasPose = false;
	hasScale = false;

	poseJoint = "";
	scaleJoint = "";

	rotationAxis1 = Vec3f::UnitX();
	rotationAxis2 = Vec3f::UnitY();
	rotationAxis3 = Vec3f::UnitZ();

	e1 = 0;
	e2 = 1;
	e3 = 2;
}

//===========================================================================//

void Mapping::init()
{
	withPose();
	withScale();
}

//===========================================================================//

void Mapping::map(
	int index,
	const std::string &joint,
	const Vec3f &axis,
	AxisType type)
{
	axisInfos[joint].push_back(AxisInfo(axis, index, type));
	jointInfos[index] = JointInfo(joint, axis, type);

	if(++count > numParameters)
		numParameters = count;
}

//===========================================================================//

void Mapping::map(
	int index,
	const std::string &joint,
	const Vec3f &axis,
	AxisType type,
	float min,
	float max)
{
	map(index, joint, axis, type);

	mins[index] = min;
	maxs[index] = max;
}

//===========================================================================//

void Mapping::withPose(
	const std::string &joint,
	int r1, int r2, int r3)
{
	hasPose = true;
	poseJoint = joint;

	rotationAxis1 = Vec3f::Unit(r1);
	rotationAxis2 = Vec3f::Unit(r2);
	rotationAxis3 = Vec3f::Unit(r3);

	e1 = r1;
	e2 = r2;
	e3 = r3;

#if 1
	map(0, poseJoint, Vec3f::UnitX(), TRANSLATION_AXIS);
	map(1, poseJoint, Vec3f::UnitY(), TRANSLATION_AXIS);
	map(2, poseJoint, Vec3f::UnitZ(), TRANSLATION_AXIS);

	map(3, poseJoint, rotationAxis1, ROTATION_AXIS);
	map(4, poseJoint, rotationAxis2, ROTATION_AXIS);
	map(5, poseJoint, rotationAxis3, ROTATION_AXIS);
#else
    // debug: disable pose via extreme joint bounds
	map(0, poseJoint, Vec3f::UnitX(), TRANSLATION_AXIS, -0.0001f, 0.0001f);
	map(1, poseJoint, Vec3f::UnitY(), TRANSLATION_AXIS, -0.0001f, 0.0001f);
	map(2, poseJoint, Vec3f::UnitZ(), TRANSLATION_AXIS, -0.0001f, 0.0001f);

	map(3, poseJoint, rotationAxis1, ROTATION_AXIS, -0.001f, 0.0001f);
	map(4, poseJoint, rotationAxis2, ROTATION_AXIS, -0.001f, 0.0001f);
	map(5, poseJoint, rotationAxis3, ROTATION_AXIS, -0.001f, 0.0001f);
#endif

	setFixedDof(6);
}

//===========================================================================//

void Mapping::withScale(
	const std::string &joint)
{
	hasScale = true;
	scaleJoint = joint;
}

//===========================================================================//

void Mapping::setDefaults(bool definePose, bool defineForearm)
{
	Vec3f X_AXIS(1,0,0);
	Vec3f Y_AXIS(0,1,0);
	Vec3f Z_AXIS(0,0,1);

	int idx;

	if(definePose)
	{
		init();
		idx = 6;
	}
	else
	{
		withScale();
		idx = 0;
	}

	if(defineForearm)
	{
		map(idx++, "LeftHandForearm", Z_AXIS, ROTATION_AXIS, -0.5, 0.5);
		map(idx++, "LeftHandForearm", X_AXIS, ROTATION_AXIS, -1.0, 1.0);
		setFixedDof(8);
	}

	map(idx++, "LeftHandThumb1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandThumb1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandThumb2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandThumb3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandIndex1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandIndex1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandIndex2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandIndex3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandMiddle1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandMiddle1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandMiddle2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandMiddle3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandRing1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandRing1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandRing2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandRing3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandPinky1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandPinky1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandPinky2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandPinky3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
}

//===========================================================================//

void Mapping::setDefaultsR(bool definePose, bool defineForearm)
{
	Vec3f X_AXIS(1,0,0);
	Vec3f Y_AXIS(0,-1,0);
	Vec3f Z_AXIS(0,0,-1);

	int idx;

	if(definePose)
	{
		init();
		idx = 6;
	}
	else
	{
		withScale();
		idx = 0;
	}

	if(defineForearm)
	{
		map(idx++, "LeftHandForearm", Z_AXIS, ROTATION_AXIS, -0.5, 0.5);
		map(idx++, "LeftHandForearm", X_AXIS, ROTATION_AXIS, -1.0, 1.0);
		setFixedDof(8);
	}

	map(idx++, "LeftHandThumb1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandThumb1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandThumb2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandThumb3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandIndex1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandIndex1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandIndex2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandIndex3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandMiddle1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandMiddle1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandMiddle2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandMiddle3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandRing1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandRing1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandRing2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandRing3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "LeftHandPinky1", Z_AXIS, ROTATION_AXIS, -0.2, 0.2);
	map(idx++, "LeftHandPinky1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandPinky2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "LeftHandPinky3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
}

//===========================================================================//

Vec3f Mapping::getRotationAxis(int i)
{
	if(i == 0)
		return rotationAxis1;
	if(i == 1)
		return rotationAxis2;
	if(i == 2)
		return rotationAxis3;
	else
		return Vec3f(0,0,0);
}

//===========================================================================//

Vector3i Mapping::getRotationAxes()
{
	return Vector3i(e1, e2, e3);
}

//===========================================================================//

Vec3f Mapping::toEulerAngles(const Mat3f &m)
{
	return ik::toEulerAngles(m, e1, e2, e3);
}

//===========================================================================//

Vec3f Mapping::toEulerAngles(const Mat4f &m)
{
	return ik::toEulerAngles(m, e1, e2, e3);
}

//===========================================================================//

Mat3f Mapping::fromEulerAngles(const Vec3f &v)
{
	return ik::fromEulerAngles(v, e1, e2, e3);
}

//===========================================================================//

Mat3f Mapping::fromEulerAngles(float a, float b, float c)
{
	return ik::fromEulerAngles(a, b, c, e1, e2, e3);
}

//===========================================================================//

// pca usage:
// - call doPCA with database, desired pc space dimension and fixed dof.
// - fixed dof are the parts of the kinematic model that are unaffected by the
//   pc space mapping because they are not contained in the database. they must
//   be defined as the first elements of the joint angle / pc space vectors.
//   for me, these are the global translation / rotation parameters.
// - use fromPC and toPC to convert between joint angle and pc space.

void Mapping::doPCA(
	const vector<vector<float> > &data,
	bool setLimits, size_t dof, size_t fDofData)
{
	pca = PCA(data, dof, numFixedDof, fDofData);

	enablePCA = true;
	numDof = dof;

	if(setLimits)
		setParameterLimits(data, dof, fDofData);
}

//===========================================================================//

void Mapping::adaptPCA(
	const vector<float> &posture)
{
	VecXf sample = pca.adapt(posture);
	numDof = pca.numDof;

	// update parameter limits for corrective dofs
	if(pca.isValid(sample))
	{
		const vector<float> &s = toPC(posture);

		for(size_t i = pca.as.cols(); i < s.size(); ++i)
		{
			if(minsPC.find(i) == minsPC.end())
				minsPC[i] = s[i];
			else if(minsPC[i] > s[i])
				minsPC[i] = s[i];

			if(maxsPC.find(i) == maxsPC.end())
				maxsPC[i] = s[i];
			else if(maxsPC[i] < s[i])
				maxsPC[i] = s[i];
		}
	}
}

//===========================================================================//

void Mapping::resetPCA()
{
	if(!enablePCA)
		return;
	pca.reset();
	numDof = pca.numDof;
}

//===========================================================================//

void Mapping::expandPCA()
{
	if(!enablePCA)
		return;
	pca.expand();
	numDof = pca.numDof;
}

//===========================================================================//

void resetMap(map<int, float> &m, size_t f, size_t n, float val)
{
	for(size_t i = f; i < f + n; ++i)
		m[i] = val;
}

//===========================================================================//

void Mapping::setParameterLimits(
	const vector<vector<float> > &data,
	size_t dof, size_t fDofData)
{
	size_t num = data.size();
	if(!num)
		return;

	size_t dim = data[0].size() - fDofData;
	if(!dof)
		dof = dim;

	resetMap(mins, numFixedDof, dim, numeric_limits<float>::max());
	resetMap(maxs, numFixedDof, dim, numeric_limits<float>::lowest());

	if(enablePCA)
	{
		resetMap(minsPC, numFixedDof, dof, numeric_limits<float>::max());
		resetMap(maxsPC, numFixedDof, dof, numeric_limits<float>::lowest());

		for(int i = 0; i < numFixedDof; ++i)
		{
			if(hasLimits(i))
			{
				minsPC[i] = mins[i];
				maxsPC[i] = maxs[i];
			}
		}
	}

	for(size_t i = 0; i < num; ++i)
	{
		vector<float> dat(numFixedDof + dim, 0);
		for(size_t j = numFixedDof; j < numFixedDof + dim; ++j)
			dat[j] = data[i][j - (numFixedDof - fDofData)];

		for(size_t j = numFixedDof; j < numFixedDof + dim; ++j)
		{
			float val = dat[j];

			if(val < mins[j])
				mins[j] = val;

			if(val > maxs[j])
				maxs[j] = val;
		}

		if(enablePCA)
		{
			const vector<float> &pc = toPC(dat);

			for(size_t j = numFixedDof; j < numFixedDof + dof; ++j)
			{
				float val = pc[j];

				if(val < minsPC[j])
					minsPC[j] = val;

				if(val > maxsPC[j])
					maxsPC[j] = val;
			}
		}
	}

	//-- debug output

	cout << "new parameter limits:" << endl;
	cout << " joint angle space:" << endl;

	for(size_t i = numFixedDof; i < numFixedDof + dim; ++i)
		cout << "  " << i << ": [" << mins[i] << ", " << maxs[i] << "]" << endl;
	
	if(enablePCA)
	{
		cout << " pca space:" << endl;
		for(size_t i = numFixedDof; i < numFixedDof + dof; ++i)
			cout << "  " << i << ": [" << minsPC[i] << ", " << maxsPC[i] << "]" << endl;
	}
	
	cout << endl;

	//--
}

//===========================================================================//

std::vector<float> Mapping::toPC(const vector<float> &param, size_t newDof) const
{
	if(!enablePCA)
		return param;
	else
		return pca.toPC(param, newDof);
}

//===========================================================================//

std::vector<float> Mapping::fromPC(const vector<float> &param, size_t newDof) const
{
	if(!enablePCA)
		return param;
	else
		return pca.fromPC(param, newDof);
}

//===========================================================================//

MatXf Mapping::getPCs(size_t dof) const
{
	return pca.getPCs(dof);
}

//===========================================================================//
	
std::vector<float> Mapping::getEigenvalues()
{
	return pca.eig;
}

//===========================================================================//
	
std::vector<float> Mapping::getMidpoint()
{
	return pca.mid;
}

//===========================================================================//

std::vector<AxisInfo> Mapping::getAxisInfos(const std::string &joint) const
{
    std::map< std::string, std::vector<AxisInfo> >::const_iterator cit;
    cit = axisInfos.find(joint);
    if(cit != axisInfos.end())
        return cit->second;
	else
		return std::vector<AxisInfo>();
}

//===========================================================================//

JointInfo Mapping::getJointInfo(int index) const
{
    std::map< int, JointInfo >::const_iterator cit;
    cit = jointInfos.find(index);
    if(cit != jointInfos.end())
        return cit->second;
	else
		return JointInfo();
}

//===========================================================================//

std::vector<float> Mapping::constrain(const std::vector<float> &param) const
{
	vector<float> result(param.size());

	for(size_t i = 0; i < param.size(); ++i)
	{
		float qmin = getMin(i);
		float qmax = getMax(i);

		float q = param[i];
		float e = 0.001;

		if(q < qmin)
			result[i] = qmin + e;
		else if(q > qmax)
			result[i] = qmax - e;
		else
			result[i] = q;
	}

	return result;
}

//===========================================================================//

float Mapping::constrain(int index, float current, float delta)
{
	if(mins.find(index) == mins.end() || maxs.find(index) == maxs.end())
	{
		return delta;
	}

	if(current + delta > maxs[index])
	{
		return maxs[index] - current;
	}

	if(current + delta < mins[index])
	{
		return current - mins[index];
	}

	return delta;
}

//===========================================================================//

float Mapping::getMin(int index) const
{
    std::map<int, float>::const_iterator cit;
    cit = mins.find(index);
    if(cit != mins.end())
        return cit->second;
	else
		//return std::numeric_limits<float>::min();
		return -std::numeric_limits<float>::max();
}

//===========================================================================//

float Mapping::getMax(int index) const
{
    std::map<int, float>::const_iterator cit;
    cit = maxs.find(index);
    if(cit != maxs.end())
        return cit->second;
	else
		return std::numeric_limits<float>::max();
}

//===========================================================================//

std::vector<float> Mapping::constrainPC(const std::vector<float> &param)
{
	vector<float> result(param.size());

	for(size_t i = 0; i < param.size(); ++i)
	{
		float qmin = getMinPC(i);
		float qmax = getMaxPC(i);

		float q = param[i];
		float e = 0.01;

		if(q < qmin)
			result[i] = qmin + e;
		else if(q > qmax)
			result[i] = qmax - e;
		else
			result[i] = q;
	}

	return result;
}

//===========================================================================//

float Mapping::constrainPC(int index, float current, float delta)
{
	if(minsPC.find(index) == minsPC.end() || maxsPC.find(index) == maxsPC.end())
	{
		return delta;
	}

	if(current + delta > maxsPC[index])
	{
		return maxsPC[index] - current;
	}

	if(current + delta < minsPC[index])
	{
		return current - minsPC[index];
	}

	return delta;
}

//===========================================================================//

float Mapping::getMinPC(int index) const
{
    std::map<int, float>::const_iterator cit;
    cit = minsPC.find(index);
    if(cit != minsPC.end())
        return cit->second;
	else
		return std::numeric_limits<float>::min();
}

//===========================================================================//

float Mapping::getMaxPC(int index) const
{
    std::map<int, float>::const_iterator cit;
    cit = maxsPC.find(index);
    if(cit != maxsPC.end())
        return cit->second;
	else
		return std::numeric_limits<float>::max();
}

//===========================================================================//

int Mapping::getNumberOfParameters() const
{
	return numParameters;
}

//===========================================================================//

int Mapping::getDof() const
{
	return numDof;
}

//===========================================================================//

int Mapping::getFixedDof() const
{
	return numFixedDof;
}

//===========================================================================//

void Mapping::setFixedDof(int fDof)
{
	numFixedDof = fDof;
}

//===========================================================================//

void Mapping::setPCAEnabled(bool b)
{
	enablePCA = b;
}

//===========================================================================//

bool Mapping::getPCAEnabled() const
{
	return enablePCA && pca.pcs.size() > 0;
}

//===========================================================================//

bool Mapping::hasPoseJoint()
{
	return hasPose;
}

//===========================================================================//

bool Mapping::hasScaleJoint()
{
	return hasScale;
}

//===========================================================================//

std::string Mapping::getPoseJoint()
{
	return poseJoint;
}

//===========================================================================//

std::string Mapping::getScaleJoint()
{
	return scaleJoint;
}

//===========================================================================//

} // namespace ik

