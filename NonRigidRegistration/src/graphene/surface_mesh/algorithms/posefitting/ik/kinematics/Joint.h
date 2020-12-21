#ifndef JOINT_H
#define JOINT_H

#include <string>
#include <vector>

#include "../math/Math.h"

namespace ik {

    class Joint
    {
        public:

            Joint(
                    const std::string &name,
                    Joint *parent = NULL,
                    const Vec3f &translation = Vec3f(0,0,0));

            void update();
            void addChild(Joint *child);

            void setParent(Joint *parent);
            void setChildren(const std::vector<Joint *> &children);

            void setTranslation(const Vec3f &t);
            void setRotation(const Vec3f &axis, float angle);
            void setRotation(const Mat3f &matrix);
            void setTransformation(const ik::Mat4f &matrix);

            void translate(const Vec3f &t);
            void rotate(const Vec3f &axis, float angle);
            void transform(const Mat4f &matrix, bool order = true);

            std::string getName() const;

            Vec3f getTranslation();
            Vec3f getGlobalTranslation() const;

            Mat3f getRotation();
            Mat3f getGlobalRotation() const;

            Mat4f &getLocalTransformation();
            Mat4f &getGlobalTransformation();

            Joint *getParent() const;
            const std::vector<Joint *> &getChildren() const;

            Vec3f getEndPosition(bool force_y = true);

            bool hasChildren() {
                return !children.empty();
            }

        protected:

            std::string name;

            Mat4f local;
            Mat4f global;

            Joint *parent;
            std::vector<Joint *> children;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

} // namespace ik

#endif

