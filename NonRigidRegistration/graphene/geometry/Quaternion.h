#ifndef GRAPHENE_QUATERNION_H
#define GRAPHENE_QUATERNION_H


#include <graphene/geometry/Vector.h>
#include <cmath>


namespace graphene
{


template<typename Scalar>
class Quaternion
{
public:
    Quaternion() :
        w(Scalar(1)),x(Scalar(0)),y(Scalar(0)),z(Scalar(0))
    {}

    Quaternion(const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z) :
        w(w), x(x),y(y),z(z)
    {}

    Scalar & operator[](int i)
    {
        return *((&w)+i);
    }
    Scalar const & operator[](int i) const
    {
        return *((&w)+i);
    }

    Quaternion& operator /= (Scalar s)
    {
        w /= s;
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    Quaternion& operator *= (Scalar s)
    {
        w *= s;
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Quaternion& operator += (Quaternion<Scalar> q)
    {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }

    Quaternion& operator -= (Quaternion<Scalar> q)
    {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }

    /// access to Scalar array
    Scalar* data()
    {
        return &w;
    }

    /// access to const Scalar array
    const Scalar* data() const
    {
        return &w;
    }

    static Quaternion<Scalar> rotate(const Vector<Scalar,3>& axis, float angle);
    ///return quaternion from euler angles yaw pitch roll
    static Quaternion<Scalar> rotate(const Vector<Scalar,3>& euler_angles);

public:
    Scalar w,x,y,z;
};

template <typename Scalar>
Quaternion<Scalar>
Quaternion<Scalar>::rotate(const Vector<Scalar, 3> &axis, float angle)
{
    Quaternion<Scalar> q;

    angle = angle * (3.14159265f/180.0f);

    normalize(axis);
    float sin_a = std::sin( angle / Scalar(2) );
    float cos_a = std::cos( angle / Scalar(2) );
    q.x = axis.x * sin_a;
    q.y = axis.y * sin_a;
    q.z = axis.z * sin_a;
    q.w = cos_a;

    return normalize(q);
}

template <typename Scalar>
Quaternion<Scalar>
Quaternion<Scalar>::rotate(const Vector<Scalar, 3> &euler_angles)
{
    Quaternion<Scalar> q;

    Vector<Scalar,3> angles = euler_angles * Scalar(0.5);
    Vector<Scalar,3> c;
    c.x = std::cos(angles.x);
    c.y = std::cos(angles.y);
    c.z = std::cos(angles.z);
    Vector<Scalar,3> s;
    s.x = std::sin(angles.x);
    s.y = std::sin(angles.y);
    s.z = std::sin(angles.z);

    q.w = c.x * c.y * c.z + s.x * s.y * s.z;
    q.x = s.x * c.y * c.z - c.x * s.y * s.z;
    q.y = c.x * s.y * c.z + s.x * c.y * s.z;
    q.z = c.x * c.y * s.z - s.x * s.y * c.z;

    return q;
}


/// read the space-separated components of a vector from a stream
template <typename Scalar>
inline std::istream& operator>>(std::istream& is, Quaternion<Scalar>& q)
{
    for (int i=0; i<4; ++i)
        is >> q[i];
    return is;
}


/// output a vector by printing its space-separated compontens
template <typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar>& q)
{
    for (int i=0; i<3; ++i)
        os << q[i] << " ";
    os << q[3];
    return os;
}

template <typename Scalar>
inline Quaternion<Scalar> conjugate(const Quaternion<Scalar>& q)
{
    return Quaternion<Scalar>(q.w, -q.x, -q.y, -q.z);
}


template <typename Scalar>
inline Quaternion<Scalar> inverse(const Quaternion<Scalar>& q)
{
    return conjugate(q) / dot(q,q);
}

template <typename Scalar>
inline Scalar length (const Quaternion<Scalar>& q)
{
    return Scalar(std::sqrt(dot(q,q)));
}

template <typename Scalar>
inline Quaternion<Scalar> normalize(const Quaternion<Scalar>& q)
{
    Scalar s = length(q);
    if (s <= Scalar(0))
        return Quaternion<Scalar>();
    s = Scalar(1) / s;

    return Quaternion<Scalar>(q.w * s, q.x * s, q.y * s, q.z * s);
}


template <typename Scalar>
inline Quaternion<Scalar> operator*(const Quaternion<Scalar>& q, const Quaternion<Scalar>& p)
{
    return Quaternion<Scalar>(
      q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z,
      q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y,
      q.w * p.y + q.y * p.w + q.z * p.x - q.x * p.z,
      q.w * p.z + q.z * p.w + q.x * p.y - q.y * p.x);
}

template <typename Scalar>
inline Quaternion<Scalar> operator/(const Quaternion<Scalar>& q, const Scalar& s)
{
    return Quaternion<Scalar>(q.w/s, q.x/s, q.y/s, q.z/s);
}


template <typename Scalar>
inline Scalar dot(const Quaternion<Scalar>& q1,const Quaternion<Scalar>& q2)
{
    return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}


template <typename Scalar>
inline Quaternion<Scalar> operator*(const Quaternion<Scalar>& q, const Scalar& s)
{
    return Quaternion<Scalar>(q.w * s, q.x * s, q.y * s, q.z * s);
}
template <typename Scalar>
inline Quaternion<Scalar> operator*(const Scalar& s, const Quaternion<Scalar>& q)
{
    return q * s;
}

template <typename Scalar>
inline Vector<Scalar,3> operator*(const Quaternion<Scalar>& q, const Vector<Scalar,3>& v)
{
    Vector<Scalar,3> qvec(q.x,q.y,q.z);
    Vector<Scalar,3> c1 = cross(qvec,v);
    Vector<Scalar,3> c2 = cross(qvec,c1);
    Scalar s2(2);

    c1 *= s2 * q.w;
    c2 *= s2;

    return v + c1 + c2;
}

template <typename Scalar>
inline Vector<Scalar,3> operator*(const Vector<Scalar,3>& v, const Quaternion<Scalar>& q)
{
    return inverse(q) * v;
}


typedef Quaternion<float> Quatf;
typedef Quaternion<double> Quatd;


}


#endif // GRAPHENE_QUATERNION_H
