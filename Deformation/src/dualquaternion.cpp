#include "dualquaternion.h"

using namespace AndreasStructures;

DualQuaternion::DualQuaternion()
{
    m_real.setW(0);
    m_real.setX(0);
    m_real.setY(0);
    m_real.setZ(1);
    m_dual.setW(0);
    m_dual.setX(0);
    m_dual.setY(0);
    m_dual.setZ(0);
}

DualQuaternion::DualQuaternion(Quaternion r, Quaternion d)
{
    r.normalize();
    m_real = r;
    m_dual = d;
}

double DualQuaternion::Dot(DualQuaternion b)
{
    return this->m_real.Dot(b.m_real);
}

DualQuaternion DualQuaternion::operator*(double scale)
{
    DualQuaternion ret = *this;
    ret.m_real *= scale;
    ret.m_dual *= scale;
    return ret;
}

DualQuaternion DualQuaternion::operator+(DualQuaternion q)
{
    DualQuaternion newQ(this->m_real + q.m_real, this->m_dual + q.m_dual);
    return newQ;
}

void DualQuaternion::normalize()
{
    double mag = this->m_real.Dot(this->m_real);
    assert( mag > 0.000001 );
    this->m_real *= 1.0 / mag;
    this->m_dual *= 1.0 / mag;
}

DualQuaternion DualQuaternion::operator* (DualQuaternion q)
{
    DualQuaternion newQ(q.m_real * this->m_real, q.m_dual * this->m_real + q.m_real * this->m_dual);
    return newQ;
}

DualQuaternion DualQuaternion::getConjugate()
{
    DualQuaternion newQ(this->m_real.getConjugate(), this->m_dual.getConjugate());
    return  newQ;
}

Quaternion DualQuaternion::getRotation()
{
    return this->m_real;
}

Eigen::Vector3d DualQuaternion::getTranslation()
{
    Quaternion t = ( this->m_dual * 2.0 ) * this->m_real;
    Eigen::Vector3d translationVector( t.getX(), t.getY(), t.getZ());
    return  translationVector;
}

Eigen::Matrix4d DualQuaternion::getTransformationMatrix()
{
    Eigen::Matrix4d M;
    double w = this->m_real.getW();
    double x = this->m_real.getX();
    double y = this->m_real.getY();
    double z = this->m_real.getZ();
    // Extract rotational information
    M(0,0) = w*w + x*x - y*y - z*z;
    M(0,1) = 2*x*y + 2*w*z;
    M(0,2) = 2*x*z - 2*w*y;
    M(0,3) = 0;
    M(1,0) = 2*x*y - 2*w*z;
    M(1,1) = w*w + y*y - x*x - z*z;
    M(1,2) = 2*y*z + 2*w*x;
    M(1,3) = 0;
    M(2,0) = 2*x*z + 2*w*y;
    M(2,1) = 2*y*z - 2*w*x;
    M(2,2) = w*w + z*z - x*x - y*y;
    M(2,3) = 0;
    // Extract translation information
    Quaternion t = (this->m_dual * 2.0f) * this->m_real.getConjugate();
    M(3,0) = t.getX();
    M(3,1) = t.getY();
    M(3,2) = t.getZ();
    M(3,3) = 1;
    return M;
}
