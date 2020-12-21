//=============================================================================

#ifndef CURVATURE_H
#define CURVATURE_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/diffgeo.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


//=============================================================================


/** Compute per-vertex curvature (min,max,mean,Gaussian).
    Curvature values for boundary vertices are interpolated from their interior
    neighbors. Curvature values can be smoothed */
class Curvature_analyzer
{
public:

    /// construct with mesh to be analyzed
    Curvature_analyzer(Surface_mesh& mesh);

    // destructor
    ~Curvature_analyzer();

    /** compute curvature information for each vertex, optionally followed
        by some smoothing iterations of the curvature values */
    void analyze(unsigned int post_smoothing_steps=0);

    /** compute curvature information for each vertex, optionally followed
        by some smoothing iterations of the curvature values */
    void analyze_tensor(unsigned int post_smoothing_steps=0,
                        bool two_ring_neighborhood=false);

    /// return mean curvature
    Scalar mean_curvature(Surface_mesh::Vertex v) const
    {
        return 0.5 * (min_curvature_[v] + max_curvature_[v]);
    }

    /// return Gaussian curvature
    Scalar gauss_curvature(Surface_mesh::Vertex v) const
    {
        return min_curvature_[v] * max_curvature_[v];
    }

    /// return minimum (signed) curvature
    Scalar min_curvature(Surface_mesh::Vertex v) const
    {
        return min_curvature_[v];
    }

    /// return maximum (signed) curvature
    Scalar max_curvature(Surface_mesh::Vertex v) const
    {
        return max_curvature_[v];
    }

    /// return maximum absolute curvature
    Scalar max_abs_curvature(Surface_mesh::Vertex v) const
    {
        return std::max(fabs(min_curvature_[v]), fabs(max_curvature_[v]));
    }


private:

    /// smooth curvature values
    void smooth_curvatures(unsigned int _iterations);


private:

    Surface_mesh& mesh_;
    Surface_mesh::Vertex_property<Scalar> min_curvature_;
    Surface_mesh::Vertex_property<Scalar> max_curvature_;
};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
