//=============================================================================

#ifndef BOUNDING_BOX_2D_H
#define BOUNDING_BOX_2D_H

//=============================================================================

#include <cfloat>
#include <graphene/types.h>
#include <graphene/geometry/Vector.h>

//=============================================================================

namespace graphene {
namespace geometry {

//=============================================================================


class Bounding_box_2D
{
public:

    Bounding_box_2D(Scalar _xmin=FLT_MAX, Scalar _ymin=FLT_MAX,
                    Scalar _xmax=-FLT_MAX, Scalar _ymax=-FLT_MAX)
        : min_(_xmin, _ymin), 
          max_(_xmax, _ymax)
    {}
    
    Bounding_box_2D(const Pixel_coordinate& _min, const Pixel_coordinate& _max)
        : min_(_min), max_(_max)
    {}

    Bounding_box_2D& operator+=(const Pixel_coordinate& _p)
    {
        for (int i=0; i<2; ++i)
        {
            if      (_p[i] < min_[i])  min_[i] = _p[i];
            else if (_p[i] > max_[i])  max_[i] = _p[i];
        }
        return *this;
    }

    Bounding_box_2D& operator+=(const Bounding_box_2D& _bb)
    {
        for (int i=0; i<2; ++i)
        {
            if (_bb.min_[i] < min_[i]) min_[i] = _bb.min_[i];
            if (_bb.max_[i] > max_[i]) max_[i] = _bb.max_[i];
        }
        return *this;
    }

    Pixel_coordinate& min() { return min_; }
    const Pixel_coordinate& min() const { return min_; }
    Pixel_coordinate& max() { return max_; }
    const Pixel_coordinate& max() const { return max_; }
    
    Pixel_coordinate   center() const { return 0.5f * (min_+max_); }
    bool  is_empty() const { return (max_[0]<min_[0] || max_[1]<min_[1]); }
    Scalar    size() const { return is_empty() ? 0.0 : distance(max_, min_); }

    bool is_inside(const Pixel_coordinate &_point)
    {
        return (_point[0] >= min_[0] && _point[0] <= max_[0]
             && _point[1] >= min_[1] && _point[1] <= max_[1]);
    }


private:

    Pixel_coordinate min_, max_;
};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // BOUNDING_BOX_2D_H
//=============================================================================
