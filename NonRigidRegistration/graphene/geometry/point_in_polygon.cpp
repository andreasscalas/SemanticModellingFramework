//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/geometry/point_in_polygon.h>


//=============================================================================


namespace graphene {
namespace geometry {


//=============================================================================


int point_in_polygon(Vec2f p, std::vector<Vec2f> polygon)
{
    int i, j, c = 0;
    int nvert = polygon.size();
    double testx = p[0];
    double testy = p[1];
    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if ( ((polygon[i][1]>testy) != (polygon[j][1]>testy)) &&
             (testx < (polygon[j][0]-polygon[i][0]) * (testy-polygon[i][1]) /
              (polygon[j][1]-polygon[i][1]) + polygon[i][0]) )
            c = !c;
    }
    return c;
}


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
