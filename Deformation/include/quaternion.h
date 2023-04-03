#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Core>
namespace AndreasStructures {

    class Quaternion
    {
    public:
        Quaternion();
        Quaternion(const double angle, const double x, const double y, const double z);
        Quaternion(const Quaternion &q);
        Eigen::Matrix4d getRotationMatrix();
        double getMagnitude();
        Quaternion getConjugate();
        void normalize();
        double Dot(const Quaternion q);
        Quaternion operator*(const Quaternion q);
        Quaternion operator*(const double c);
        Quaternion operator+(const Quaternion q);
        void operator=(const Quaternion q);
        void operator*=(const double c);
        void operator*=(const Quaternion q);
        void operator+=(const Quaternion q);
        double getW() const;
        void setW(double value);
        double getX() const;
        void setX(double value);
        double getY() const;
        void setY(double value);
        double getZ() const;
        void setZ(double value);

    private:
        double w, x, y, z;
    };

}

#endif // QUATERNION_H
