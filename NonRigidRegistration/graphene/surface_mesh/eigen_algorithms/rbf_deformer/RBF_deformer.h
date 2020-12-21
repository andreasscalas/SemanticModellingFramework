//=============================================================================


#ifndef GRAPHENE_RBF_DEFORMER_H
#define GRAPHENE_RBF_DEFORMER_H


//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class RBF_deformer
{

public:

    RBF_deformer(Surface_mesh& mesh,
                 const std::vector<Point>& src_points,
                 const std::vector<Point>& tar_points);

private:

    /// evaluate RBF at position \c p
    Vec3d evaluate(const Vec3d& p) const;


    /// RBF kernel
    inline double kernel(const Vec3d& c, const Vec3d& x) const
    {
        const double r = norm(x-c);

        /* return r*r*r; // triharmonic */
        return r; // spline
    }


private:

    std::vector<Vec3d> centers_;

    std::vector<Vec3d> weights_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_RBF_DEFORMER_H defined
//=============================================================================
