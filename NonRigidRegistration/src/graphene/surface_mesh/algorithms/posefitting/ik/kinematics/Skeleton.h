#ifndef SKELETON_H
#define SKELETON_H

#include <map>
#include <vector>

#include "../math/Math.h"
#include "Joint.h"
#include "Effector.h"
#include "Mapping.h"

namespace ik {

    class Skeleton
    {
        public:

            Skeleton(Joint *root, const Mapping &mapping);
            Skeleton();

            ~Skeleton();

            Skeleton *deepCopy();

            void move(const std::vector<float> &parameters);
            void set(const std::vector<float> &parameters);
            void moveEffectors();

            void rotate(const Vec3f &axis, float angle);
            void translate(const Vec3f &translation);
            void transform(const Mat4f &matrix);
            void scale(float value);

            void setRotation(const Mat3f &matrix);
            void setRotation(const Vec3f &axis, float angle);
            void setTranslation(const Vec3f &translation);
            void setTransformation(const Mat4f &matrix);
            void setScale(float value);
            void setScaleValue(float value);

        public:

            void transformJoints(const std::vector<float> &parameters);

            void setRoot(Joint *root);
            void setForwardAxis(const Vec3f &xyz);

            void setPoseJoint(
                    const std::string &name,
                    int tx=0, int ty=1, int tz=2,
                    int rx=3, int ry=4, int rz=5);

            void setScaleJoint(
                    const std::string &name);

            void setInitialTransformation(const std::string &jointName, const Mat4f &m);
            void setInitialTransformations(const aligned_map<std::string, Mat4f>::type &ts);
            void setInitialTransformations();
            void setInitialTranslations();
            void reset(bool reinit = false);
            void resetPosture(bool reinit = false);

            aligned_map<std::string, Mat4f>::type getInitialTransformations();
            Mat4f getInitialTransformation(const std::string &jointName);

            void addJoint(Joint *joint);
            void addEffector(Effector *effector);
            void addEffector(Effector *effector, size_t index);
            void clearJoints();

            void removeEffector(size_t index);
            void initEffectors(size_t n);
            void clearEffectors();
            bool hasEffectors(const std::string &joint);

            Joint *getRoot();
            Joint *getJoint(const std::string &joint);
            Joint *getJoint(int index)
            {
                return joints_[index];
            }

            bool isRoot(Joint *joint) {
                return joint == root;
            }

            bool isStructural(Joint *joint) {
                return joint == root
                    || joint == joints[poseJoint]
                    || joint == joints[scaleJoint];
            }

        public:

            void setGlobalTranslation(Joint *joint, const Vec3f &target, bool undo = true);
            void translateGlobal(Joint *joint, const Vec3f &target);
            void alignRotation(Joint *joint, bool wristHack = false);

        public:

            Joint *getPoseJoint();
            std::vector<int> getPoseJointIndices();
            bool hasPoseJoint();
            Vec3f getEulerAngles();
            std::vector<Vec3f> getPhalanxEndpoints();

            Joint *getScaleJoint();
            float getScale();
            float getScaleMagnitude();

            std::map<std::string, Joint *> &getJoints();
            std::map<std::string, std::vector<Effector *> > &getEffectors();
            std::vector<Effector *> &getEffectors(const std::string &joint);

            std::vector<Joint *> &getJoints_();
            std::vector<Effector *> &getEffectors_();

            int getNumberOfJoints();
            int getNumberOfEffectors();
            float getEffectorDistance();

            std::vector<float> getCurrentParameters();
            std::vector<float> getCurrentPosture();
            std::vector<float> getUpdatedParameters(
                    const std::vector<float> &ps,
                    const std::vector<float> &dt,
                    bool psDim = false);

            void setMapping(const Mapping &mapping);
            Mapping &getMapping();

        public:

            void generateIDs() {
                size_t numjs = joints_.size();

                for(size_t i = 0; i < numjs; ++i)
                {
                    size_t id = i;
                    const std::string &name = joints_[i]->getName();

                    ids[id] = name;
                    ids2[name] = id;
                }
            }

            bool hasIDs() {
                return !ids.empty() && !ids2.empty();
            }

            std::string getJointName(size_t id) {
                assert(id>=0);
                return ids[id];
            }

            size_t getID(const std::string &n) {
                return ids2[n];
            }

        public:

            static Skeleton *leftHand();
            static Skeleton *leftArm();

            static Skeleton *rightHand();
            static Skeleton *rightArm();

        protected:

            Mapping mapping;

            Joint *root;

            std::string poseJoint;
            std::string scaleJoint;

            int tx, ty, tz, rx, ry, rz;
            float scaleVal;

            std::map<std::string, Joint *> joints;
            std::map<std::string, std::vector<Effector *> > effectors;

            std::vector<Joint *> joints_;
            std::vector<Effector *> effectors_;

            int numJoints;
            int numEffectors;

            Vec3f forward;

            std::vector<float> current;

            aligned_map<std::string, Mat4f>::type initialTransformations;

            // joint ids
            std::map<size_t, std::string> ids;  // id => name
            std::map<std::string, size_t> ids2; // name => id

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

} // namespace ik

#endif

