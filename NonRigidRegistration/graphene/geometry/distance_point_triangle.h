//=============================================================================

#ifndef DISTANCE_POINT_TRIANGLE_H
#define DISTANCE_POINT_TRIANGLE_H


//== INCLUDES =================================================================

#include <graphene/types.h>


//=============================================================================


namespace graphene {
namespace geometry {


//=============================================================================


/// distance from point _p to line segment (_v0,_v1)
Scalar dist_point_linesegment(const Point& _p,
                              const Point& _v0,
                              const Point& _v1,
                              Point&       _nearest_point );


/// distance from point _p to triangle (_v0, _v1, _v2)
Scalar dist_point_triangle(const Point& _p,
                           const Point& _v0,
                           const Point& _v1,
                           const Point& _v2,
                           Point&       _nearest_point);


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
