#include "quaternion.h"

#include <math.h>

using namespace AndreasStructures;

Quaternion::Quaternion()
{
    w = 1;
    x = 0;
    y = 0;
    z = 0;
}

Quaternion::Quaternion(const double angle, const double x, const double y, const double z)
{
    this->w = cos(angle/2);
    this->x = x * sin(angle/2);
    this->y = y * sin(angle/2);
    this->z = z * sin(angle/2);
}

Quaternion::Quaternion(const Quaternion &q)
{
    this->w = q.getW();
    this->x = q.getX();
    this->y = q.getY();
    this->z = q.getZ();
}

Eigen::Matrix4d Quaternion::getRotationMatrix()
{
    Eigen::Matrix4d r;
    r << pow(w,2) + pow(x,2) - pow(y,2) - pow(z,2), 2 * x * y - 2 * w * z,                     2 * x * z + 2 * w * y,                       0,
         2 * x * y + 2 * w * z,                     pow(w,2) - pow(x,2) + pow(y,2) - pow(z,2), 2 * y * z + 2 * w * x,                       0,
         2 * x * z - 2 * w * y,                     2 * y * z - 2 * w * x,                     pow(w,2) - pow(x,2) - pow(y,2) + pow(z,2),   0,
         0,                                         0,                                         0,                                           1;
    return r;
}

double Quaternion::getMagnitude()
{
    return sqrt(pow(w,2) + pow(x,2) + pow(y,2) + pow(z,2));
}

Quaternion Quaternion::getConjugate()
{
    Quaternion q;
    q.setW(w);
    q.setX(-x);
    q.setY(-y);
    q.setZ(-z);
    return q;
}

void Quaternion::normalize()
{
    double m = getMagnitude();
    w /= m;
    x /= m;
    y /= m;
    z /= m;

}

double Quaternion::Dot(const Quaternion q)
{
    return this->x * q.x + this->y * q.y + this->z * q.z + this->w * q.w;
}

Quaternion Quaternion::operator*(const Quaternion q)
{
    double newW = this->w * q.getW() - this->x * q.getX() - this->y * q.getY() - this->z * q.getZ();
    double newX = this->w * q.getX() + this->x * q.getW() + this->y * q.getZ() - this->z * q.getY();
    double newY = this->w * q.getY() - this->x * q.getZ() + this->y * q.getW() + this->z * q.getX();
    double newZ = this->w * q.getZ() + this->x * q.getY() - this->y * q.getX() + this->z * q.getW();
    Quaternion newQ;
    newQ.setW(newW);
    newQ.setX(newX);
    newQ.setY(newY);
    newQ.setZ(newZ);
    return newQ;
}

Quaternion Quaternion::operator*(const double c)
{
    this->w *= c;
    this->x *= c;
    this->y *= c;
    this->z *= c;
    return *this;
}

Quaternion Quaternion::operator+(const Quaternion q)
{
    this->w += q.getW();
    this->x += q.getX();
    this->y += q.getY();
    this->z += q.getZ();
    return *this;
}

void Quaternion::operator=(const Quaternion q)
{
    this->w = q.getW();
    this->x = q.getX();
    this->y = q.getY();
    this->z = q.getZ();
}

void Quaternion::operator*=(const double c)
{
    this->w *= c;
    this->x *= c;
    this->y *= c;
    this->z *= c;
}

void Quaternion::operator*=(const Quaternion q)
{
    double newW = this->w * q.getW() - this->x * q.getX() - this->y * q.getY() - this->z * q.getZ();
    double newX = this->w * q.getX() + this->x * q.getW() + this->y * q.getZ() - this->z * q.getY();
    double newY = this->w * q.getY() - this->x * q.getZ() + this->y * q.getW() + this->z * q.getX();
    double newZ = this->w * q.getZ() + this->x * q.getY() - this->y * q.getX() + this->z * q.getW();
    this->w = newW;
    this->x = newX;
    this->y = newY;
    this->z = newZ;
}

void Quaternion::operator+=(const Quaternion q)
{
    this->w += q.getW();
    this->x += q.getX();
    this->y += q.getY();
    this->z += q.getZ();
}

double Quaternion::getW() const
{
    return w;
}

double Quaternion::getX() const
{
    return x;
}

double Quaternion::getY() const
{
    return y;
}

double Quaternion::getZ() const
{
    return z;
}

void Quaternion::setZ(double value)
{
    z = value;
}

void Quaternion::setY(double value)
{
    y = value;
}

void Quaternion::setX(double value)
{
    x = value;
}

void Quaternion::setW(double value)
{
    w = value;
}
