//=============================================================================

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

//=============================================================================

#include <cfloat>
#include <graphene/types.h>
#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Geometry_representation.h>

//=============================================================================

namespace graphene {
namespace geometry {

//=============================================================================


/// \addtogroup geometry
/// @{


//=============================================================================


/// Simple class for representing a bounding box
class Bounding_box
{
public:

    Bounding_box(Scalar _xmin=FLT_MAX, Scalar _ymin=FLT_MAX, Scalar _zmin=FLT_MAX,
                 Scalar _xmax=-FLT_MAX, Scalar _ymax=-FLT_MAX, Scalar _zmax=-FLT_MAX)
        : min_(_xmin, _ymin, _zmin), 
          max_(_xmax, _ymax, _zmax)
    {}
    
    Bounding_box(const Vec3f& _min, const Vec3f& _max)
        : min_(_min), max_(_max)
    {}

    Bounding_box(Geometry_representation& geometry)
        : min_(FLT_MAX), max_(-FLT_MAX)
    {
        for (auto p : geometry.points())
        {
            min_.minimize(p);
            max_.maximize(p);
        }
    }

    Bounding_box& operator+=(const Point& _p)
    {
        for (int i=0; i<3; ++i)
        {
            if      (_p[i] < min_[i])  min_[i] = _p[i];
            else if (_p[i] > max_[i])  max_[i] = _p[i];
        }
        return *this;
    }

    Bounding_box& operator+=(const Bounding_box& _bb)
    {
        for (int i=0; i<3; ++i)
        {
            if (_bb.min_[i] < min_[i]) min_[i] = _bb.min_[i];
            if (_bb.max_[i] > max_[i]) max_[i] = _bb.max_[i];
        }
        return *this;
    }

    Point& min() { return min_; }
    Point& max() { return max_; }
    
    Point   center() const { return 0.5f * (min_+max_); }
    bool  is_empty() const { return (max_[0]<min_[0] || max_[1]<min_[1] || max_[2]<min_[2]); }
    Scalar    size() const { return is_empty() ? 0.0 : distance(max_, min_); }

    bool is_inside(const Point &_point)
    {
        return (_point.x >= min_.x && _point.x <= max_.x
             && _point.y >= min_.y && _point.y <= max_.y
             && _point.z >= min_.z && _point.z <= max_.z);
    }


private:
    
    Point min_, max_;
};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // BOUNDING_BOX_H
//=============================================================================
