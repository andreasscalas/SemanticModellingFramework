//=============================================================================


#ifndef GRAPHENE_SUBSAMPLING_H
#define GRAPHENE_SUBSAMPLING_H


//== INCLUDES =================================================================


#include <graphene/geometry/Point_set.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================


class
Subsampling
{
public:

    Subsampling(Point_set& point_set);

    ~Subsampling();

    void subsample(float grid_size);

private:

    void subsampling_voxelgrid_hash( float grid_size );


private:

    Point_set& point_set_;



};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SUBSAMPLING_H
//=============================================================================
