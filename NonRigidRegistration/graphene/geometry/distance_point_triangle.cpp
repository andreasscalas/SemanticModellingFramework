//== INCLUDES =================================================================

#include <graphene/geometry/distance_point_triangle.h>
#include <math.h>
#include <float.h>


//== IMPLEMENTATION ==========================================================

namespace graphene {
namespace geometry {

Scalar dist_point_linesegment(const Point& _p,
                              const Point& _v0,
                              const Point& _v1,
                              Point&       _nearest_point)
{
    Point  d1(_p-_v0);
    Point  d2(_v1-_v0);
    Point  min_v(_v0);
    Scalar t = dot(d2,d2);

    if (t > FLT_MIN)
    {
        t = dot(d1,d2)/t;
        if      (t > 1.0)  d1 = _p - (min_v = _v1);
        else if (t > 0.0)  d1 = _p - (min_v = _v0 + d2*t);
    }

    _nearest_point = min_v;
    return norm(d1);
}


//-----------------------------------------------------------------------------


Scalar dist_point_triangle(const Point& _p,
                           const Point& _v0,
                           const Point& _v1,
                           const Point& _v2,
                           Point&       _nearest_point)
{
    Point  v0v1 = _v1 - _v0;
    Point  v0v2 = _v2 - _v0;
    Point  n = cross(v0v1,v0v2); // not normalized !
    Scalar d = sqrnorm(n);


    // Check if the triangle is degenerated -> measure dist to line segments
    if (fabs(d) < FLT_MIN)
    {
        Point  q, qq;
        Scalar d, dd(FLT_MAX);

        dd = dist_point_linesegment(_p, _v0, _v1, qq);

        d = dist_point_linesegment(_p, _v1, _v2, q);
        if (d < dd) { dd = d; qq = q; }

        d = dist_point_linesegment(_p, _v2, _v0, q);
        if (d < dd) { dd = d; qq = q; }

        _nearest_point = qq;
        return dd;
    }


    Scalar invD = 1.0 / d;
    Point  v1v2 = _v2; v1v2 -= _v1;
    Point  v0p = _p; v0p -= _v0;
    Point  t = cross(v0p,n);
    Scalar a = dot(t,v0v2) * -invD;
    Scalar b = dot(t,v0v1) * invD;
    Scalar s01, s02, s12;


    // Calculate the distance to an edge or a corner vertex
    if (a < 0)
    {
        s02 = dot(v0v2,v0p) / sqrnorm(v0v2);
        if (s02 < 0.0)
        {
            s01 = dot(v0v1,v0p) / sqrnorm(v0v1);
            if (s01 <= 0.0)
            {
                v0p = _v0;
            }
            else if (s01 >= 1.0)
            {
                v0p = _v1;
            }
            else
            {
                (v0p = _v0) += (v0v1 *= s01);
            }
        }
        else if (s02 > 1.0)
        {
            s12 = dot(v1v2,( _p - _v1 )) / sqrnorm(v1v2);
            if (s12 >= 1.0)
            {
                v0p = _v2;
            }
            else if (s12 <= 0.0)
            {
                v0p = _v1;
            }
            else
            {
                (v0p = _v1) += (v1v2 *= s12);
            }
        }
        else
        {
            (v0p = _v0) += (v0v2 *= s02);
        }
    }


    // Calculate the distance to an edge or a corner vertex
    else if (b < 0.0)
    {
        s01 = dot(v0v1,v0p) / sqrnorm(v0v1);
        if (s01 < 0.0)
        {
            s02 = dot(v0v2,v0p) / sqrnorm(v0v2);
            if (s02 <= 0.0)
            {
                v0p = _v0;
            }
            else if (s02 >= 1.0)
            {
                v0p = _v2;
            }
            else
            {
                (v0p = _v0) += (v0v2 *= s02);
            }
        }
        else if (s01 > 1.0)
        {
            s12 = dot(v1v2,( _p - _v1 )) / sqrnorm(v1v2);
            if (s12 >= 1.0)
            {
                v0p = _v2;
            }
            else if (s12 <= 0.0)
            {
                v0p = _v1;
            }
            else
            {
                (v0p = _v1) += (v1v2 *= s12);
            }
        }
        else
        {
            (v0p = _v0) += (v0v1 *= s01);
        }
    }


    // Calculate the distance to an edge or a corner vertex
    else if (a+b > 1.0)
    {
        s12 = dot(v1v2,( _p - _v1 )) / sqrnorm(v1v2);
        if (s12 >= 1.0)
        {
            s02 = dot(v0v2,v0p) / sqrnorm(v0v2);
            if (s02 <= 0.0)
            {
                v0p = _v0;
            }
            else if (s02 >= 1.0)
            {
                v0p = _v2;
            }
            else
            {
                (v0p = _v0) += (v0v2 *= s02);
            }
        }
        else if (s12 <= 0.0)
        {
            s01 = dot(v0v1,v0p) / sqrnorm(v0v1);
            if (s01 <= 0.0)
            {
                v0p = _v0;
            }
            else if (s01 >= 1.0)
            {
                v0p = _v1;
            }
            else
            {
                (v0p = _v0) += (v0v1 *= s01);
            }
        }
        else
        {
            (v0p = _v1) += (v1v2 *= s12);
        }
    }


    // Calculate the distance to an interior point of the triangle
    else
    {
        n *= (dot(n,v0p) * invD);
        (v0p = _p) -= n;
    }


    _nearest_point = v0p;
    v0p -= _p;
    return norm(v0p);
}


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
