#ifndef DUALQUATERNION_H
#define DUALQUATERNION_H

#include <quaternion.h>
#include <Eigen/Core>

namespace AndreasStructures {


    class DualQuaternion
    {
    public:
        DualQuaternion();
        DualQuaternion(Quaternion r, Quaternion d);
        double Dot(DualQuaternion b);
        DualQuaternion operator* (double scale);
        DualQuaternion operator* (DualQuaternion q);
        DualQuaternion operator+ (DualQuaternion q);
        void normalize();
        DualQuaternion getConjugate();
        Quaternion getRotation();
        Eigen::Vector3d getTranslation();
        Eigen::Matrix4d getTransformationMatrix();


    private:
        Quaternion m_real;
        Quaternion m_dual;

    };
}
#endif // DUALQUATERNION_H
