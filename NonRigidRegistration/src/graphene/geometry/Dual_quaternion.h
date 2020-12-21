#ifndef GRAPHENE_DUAL_QUATERNION_H
#define GRAPHENE_DUAL_QUATERNION_H


#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Quaternion.h>
#include <cmath>


namespace graphene
{


//pvonneumanncosel DUAL QUATERNION CLASS
template<typename Scalar>
class Dual_quaternion
{
public:
    Dual_quaternion() :
        q0(),q1()
    {}

    Dual_quaternion(const Quaternion<Scalar> &qu0, const Quaternion<Scalar> &qu1) :
        q0(qu0),
        q1(qu1)
    {
    }

    Dual_quaternion(const Scalar &w0, const Scalar &x0, const Scalar &y0, const Scalar &z0,
               const Scalar &w1, const Scalar &x1, const Scalar &y1, const Scalar &z1) :
            q0(w0,x0,y0,z0), q1(w1,x1,y1,z1)
        {}

    Dual_quaternion& operator *= (Scalar s)
    {
        q0 *= s;
        q1 *= s;

        return *this;
    }

    Dual_quaternion& operator += (Dual_quaternion<Scalar> q)
    {
        q0 += q.q0;
        q1 += q.q1;

        return *this;
    }

    Dual_quaternion& operator -= (Dual_quaternion<Scalar> q)
    {
        q0 -= q.q0;
        q1 -= q.q1;

        return *this;
    }

    /// access to Scalar array
    Scalar* data()
    {
        return q0.data();
    }

    /// access to const Scalar array
    const Scalar* data() const
    {
        return q0.data();
    }

public:
    Quaternion<Scalar> q0, q1;
};

typedef Dual_quaternion<float> DQuatf;

template <typename Scalar>
inline Dual_quaternion<Scalar> operator*(const Dual_quaternion<Scalar>& q, const Scalar& s)
{
    return Dual_quaternion<Scalar>(q.q0 * s, q.q1 * s);
}
template <typename Scalar>
inline Dual_quaternion<Scalar> operator*(const Scalar& s, const Dual_quaternion<Scalar>& q)
{
    return q * s;
}


//This function has been coded following the pseudocode Kavan proposed.
template <typename Scalar>
void
dq_to_mat4(const Dual_quaternion<Scalar>& dq_in, Mat4<Scalar> &m_out)
{

    float w = dq_in.q0.w, x = dq_in.q0.x, y = dq_in.q0.y, z = dq_in.q0.z;
    float t0 = dq_in.q1.w, t1 = dq_in.q1.x, t2 = dq_in.q1.y, t3 = dq_in.q1.z;

    m_out(0,0) = w*w + x*x - y*y - z*z; m_out(0,1) = 2*x*y - 2*w*z; m_out(0,2) = 2*x*z + 2*w*y;
    m_out(1,0) = 2*x*y + 2*w*z; m_out(1,1) = w*w + y*y - x*x - z*z; m_out(1,2) = 2*y*z - 2*w*x;
    m_out(2,0) = 2*x*z - 2*w*y; m_out(2,1) = 2*y*z + 2*w*x; m_out(2,2) = w*w + z*z - x*x - y*y;

    m_out(0,3) = -2*t0*x + 2*w*t1 - 2*t2*z + 2*y*t3;
    m_out(1,3) = -2*t0*y + 2*t1*z - 2*x*t3 + 2*w*t2;
    m_out(2,3) = -2*t0*z + 2*x*t2 + 2*w*t3 - 2*t1*y;

    m_out(3,0) = 0;
    m_out(3,1) = 0;
    m_out(3,2) = 0;
    m_out(3,3) = 1;
}

template <typename Scalar>
void
mat4_to_dualquat(const Mat4<Scalar>& m_in, Dual_quaternion<Scalar>& dq_out)
{

    const Scalar fourXSquaredMinus1 = m_in(0,0) - m_in(1,1) - m_in(2,2);
    const Scalar fourYSquaredMinus1 = m_in(1,1) - m_in(0,0) - m_in(2,2);
    const Scalar fourZSquaredMinus1 = m_in(2,2) - m_in(0,0) - m_in(1,1);
    const Scalar fourWSquaredMinus1 = m_in(0,0) + m_in(1,1) + m_in(2,2);

    int biggestIndex = 0;
    Scalar fourBiggestSquaredMinus1 = fourWSquaredMinus1;
    if(fourXSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourXSquaredMinus1;
        biggestIndex = 1;
    }
    if(fourYSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourYSquaredMinus1;
        biggestIndex = 2;
    }
    if(fourZSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourZSquaredMinus1;
        biggestIndex = 3;
    }

    Scalar biggestVal = sqrt(fourBiggestSquaredMinus1 + Scalar(1)) * Scalar(0.5);
    Scalar mult = Scalar(0.25) / biggestVal;

    switch(biggestIndex)
    {
    case 0:
        dq_out.q0.w = biggestVal;
        dq_out.q0.x = (m_in(2,1) - m_in(1,2)) * mult;
        dq_out.q0.y = (m_in(0,2) - m_in(2,0)) * mult;
        dq_out.q0.z = (m_in(1,0) - m_in(0,1)) * mult;
        break;
    case 1:
        dq_out.q0.w = (m_in(2,1) - m_in(1,2)) * mult;
        dq_out.q0.x = biggestVal;
        dq_out.q0.y = (m_in(1,0) + m_in(0,1)) * mult;
        dq_out.q0.z = (m_in(0,2) + m_in(2,0)) * mult;
        break;
    case 2:
        dq_out.q0.w = (m_in(0,2) - m_in(2,0)) * mult;
        dq_out.q0.x = (m_in(1,0) + m_in(0,1)) * mult;
        dq_out.q0.y = biggestVal;
        dq_out.q0.z = (m_in(2,1) + m_in(1,2)) * mult;
        break;
    case 3:
        dq_out.q0.w = (m_in(1,0) - m_in(0,1)) * mult;
        dq_out.q0.x = (m_in(0,2) + m_in(2,0)) * mult;
        dq_out.q0.y = (m_in(2,1) + m_in(1,2)) * mult;
        dq_out.q0.z = biggestVal;
        break;

    default:
        assert(false);
        break;
    }

    dq_out.q0 = normalize(dq_out.q0);

    dq_out.q1.w = Scalar(-0.5)* ( m_in(0,3)*dq_out.q0.x + m_in(1,3)*dq_out.q0.y + m_in(2,3)*dq_out.q0.z);
    dq_out.q1.x = Scalar(0.5)* ( m_in(0,3)*dq_out.q0.w + m_in(1,3)*dq_out.q0.z - m_in(2,3)*dq_out.q0.y);
    dq_out.q1.y = Scalar(0.5)* (-m_in(0,3)*dq_out.q0.z + m_in(1,3)*dq_out.q0.w + m_in(2,3)*dq_out.q0.x);
    dq_out.q1.z = Scalar(0.5)* ( m_in(0,3)*dq_out.q0.y - m_in(1,3)*dq_out.q0.x + m_in(2,3)*dq_out.q0.w);
}


template <typename Scalar>
Dual_quaternion<Scalar>
normalize(const Dual_quaternion<Scalar>& dq)
{
    Scalar l;
    l = length(dq.q0);
    return Dual_quaternion<Scalar>(dq.q0/l, dq.q1/l);
}

}


#endif // GRAPHENE_DUAL_QUATERNION_H
