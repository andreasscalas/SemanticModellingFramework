//== INCLUDES =================================================================


#include "Subsampling.h"
#include <graphene/geometry/Bounding_box.h>

#include <cmath>
#include <map>


//== NAMESPACE ================================================================


namespace graphene {
namespace geometry {


//== IMPLEMENTATION ===========================================================


Subsampling::
Subsampling(Point_set& point_set)
    : point_set_(point_set)
{}


//-----------------------------------------------------------------------------


Subsampling::
~Subsampling()
{}


//-----------------------------------------------------------------------------

void Subsampling::subsample(float grid_size)
{
    subsampling_voxelgrid_hash(grid_size);
}

//-----------------------------------------------------------------------------


void Subsampling::subsampling_voxelgrid_hash(float grid_size)
{
    Bounding_box bbox;

    //compute bounding box of point set to get the voxel grid's dimensions
    for (size_t i=0; i < point_set_.points().size(); ++i)
    {
        bbox += point_set_.points()[i];
    }

    Vec3i vc;
    const Vec3f& bbmin = bbox.min();
    const Vec3f& bbmax = bbox.max();


    const Vec3f d = (bbmax - bbmin);
    const Vec3i dims = (d / grid_size);

    //create vector to store point indices per voxel
    Vec3i vdims( (int) (dims[0] + 1.0f), (int) (dims[1] + 1.0f), (int) (dims[2] + 1.0f));
    /*
    std::vector< std::vector<unsigned int> > voxel_indices;
    voxel_indices.resize(  vdims[0] * vdims[1] * vdims[2] );
    */

    std::map<unsigned int, std::vector<unsigned int > > voxel_indices;

    //iterate all points and calculate their voxel coordinates
    for (size_t i=0; i < point_set_.points().size(); ++i)
    {
        const Vec3f& p = point_set_.points()[i];
        vc[0] = (int) ( (p[0] - bbmin[0]) / grid_size );
        vc[1] = (int) ( (p[1] - bbmin[1]) / grid_size );
        vc[2] = (int) ( (p[2] - bbmin[2]) / grid_size );

        const int idx = (vc[2] * vdims[0] * vdims[1]) + (vc[1] * vdims[0]) + vc[0];

        voxel_indices[idx].push_back((unsigned int) i);
    }

    std::vector<Point>  new_points;
    std::vector<Normal> new_normals;
    std::vector<Color>  new_colors;

    //iterate voxel indices and compute average of the points in each voxel
    if (point_set_.colors_.empty()) //point set has no colors
    {
        std::map<unsigned int, std::vector<unsigned int> >::const_iterator cit;
        for (cit=voxel_indices.begin(); cit != voxel_indices.end(); ++cit)
        {
            const std::vector<unsigned int> &idcs = cit->second;

            Vec3f np(0.0f);
            Vec3f nn(0.0f);
            for (size_t j=0; j < idcs.size(); ++j)
            {
                np += point_set_.points()[idcs[j]];
                nn += point_set_.normals_[idcs[j]];
            }
            np /= (float) idcs.size();
            nn /= (float) idcs.size();

            new_points.push_back(np);
            new_normals.push_back(nn);
        }
    }
    else //point has colors
    {
        std::map<unsigned int, std::vector<unsigned int> >::const_iterator cit;
        for (cit=voxel_indices.begin(); cit != voxel_indices.end(); ++cit)
        {
            const std::vector<unsigned int> &idcs = cit->second;

            Vec3f np(0.0f);
            Vec3f nn(0.0f);
            Vec3f nc(0.0f);
            for (size_t j=0; j < idcs.size(); ++j)
            {
                np += point_set_.points()[idcs[j]];
                nn += point_set_.normals_[idcs[j]];
                nc += point_set_.colors_[idcs[j]];
            }
            np /= (float) idcs.size();
            nn /= (float) idcs.size();
            nc /= (float) idcs.size();

            new_points.push_back(np);
            new_normals.push_back(nn);
            new_colors.push_back(nc);
        }
    }

    point_set_.points() = new_points;
    point_set_.normals_ = new_normals;
    point_set_.colors_  = new_colors;
}





//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
