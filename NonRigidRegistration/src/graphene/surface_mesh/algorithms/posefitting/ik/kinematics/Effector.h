#ifndef EFFECTOR_H
#define EFFECTOR_H

#include "Joint.h"

namespace ik {

	class Effector
	{
		public:

			Effector(
					Joint *joint,
					const Vec3f &position,
					const Vec3f &target,
					const Vec3f &positionNormal = Vec3f(0,0,1),
					const Vec3f &targetNormal = Vec3f(0,0,1));

			Effector() {}

			Joint *getJoint();

			Vec3f getPosition();
			Vec3f getTarget();
			Vec3f getOffset();

			Vec3f getPositionNormal();
			Vec3f getTargetNormal();

			float getRatio() { return ratio; }

			void setJoint(Joint *j);

			void setPosition(const Vec3f &p);
			void setTarget(const Vec3f &t);
			void setOffset(const Vec3f &o);
			void updateOffset();

			void setPositionNormal(const Vec3f &pn);
			void setTargetNormal(const Vec3f &tn);

			void setRatio(float r) { ratio = r; }

		protected:

			Joint *joint;

			Vec3f position;
			Vec3f target;
			Vec3f offset;

			Vec3f positionNormal;
			Vec3f targetNormal;

			float ratio;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};

} // namespace ik

#endif

