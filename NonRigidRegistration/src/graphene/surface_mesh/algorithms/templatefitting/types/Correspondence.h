//=============================================================================
#ifndef GRAPHENE_CORRESSPONDENCE_H
#define GRAPHENE_CORRESSPONDENCE_H
//=============================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>



//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================

enum Correspondence_direction { corresp_dir_mesh2ps, corresp_dir_ps2mesh, corresp_dir_mesh2ps_and_ps2mesh };


struct Correspondence
{
    Surface_mesh::Face   face;
    Point                on_template;
    Normal               on_template_n;
    Surface_mesh::Vertex on_template_vh;
    Vec3f                on_template_bc;
    Point                on_ps;
    Normal               on_ps_n;
    double               weight;
    double               distance_point2point;
    Correspondence_direction constr_dir;

    bool operator<(const Correspondence& c) const
    {
        return ( distance_point2point < c.distance_point2point ); 
    }
};



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_CORRESSPONDENCE_H
//=============================================================================
