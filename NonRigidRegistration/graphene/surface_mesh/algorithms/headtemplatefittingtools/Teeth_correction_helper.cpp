//== INCLUDES ===================================================================


#include "Teeth_correction_helper.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>
#include <graphene/geometry/registration.h>
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>

//== NAMESPACES ================================================================


namespace graphene {
namespace teeth_correction_helper {

using namespace surface_mesh;

bool correct_teeth(character::Character &inout_character,
                   const std::string &s_bodymesh,
                   const std::string &s_teeth_up,
                   const std::string &s_teeth_down,
                   const std::string &fn_sel_mouth_region)
{

    // prepare skins ------------------------------------------------------

    const std::vector<graphene::character::Surface_mesh_skin*>& skins = inout_character.skins();

    graphene::character::Surface_mesh_skin* base_mesh       = 0;
    graphene::character::Surface_mesh_skin* teeth_up_mesh   = 0;
    graphene::character::Surface_mesh_skin* teeth_down_mesh = 0;

    for (size_t i = 0; i < skins.size(); ++i)
    {
        auto skin_id = skins[i]->mesh_property<std::string>("m:id");

        if ( skin_id[0] == s_bodymesh )
        {
            base_mesh = skins[i];
        }
        else if ( skin_id[0] == s_teeth_up )
        {
            teeth_up_mesh = skins[i];
        }
        else if ( skin_id[0] == s_teeth_down )
        {
            teeth_down_mesh = skins[i];
        }
    }

    if (!base_mesh || !teeth_up_mesh || !teeth_down_mesh)
    {
        std::cerr << "correct_teeth: [ERROR] Cannot correct teeth. No base mesh or teeth meshes found!" << std::endl;
        return false;
    }


    // prepare source undeformed ------------------------------------------

    Surface_mesh::Vertex_property<Point> undefmesh_prop =
            inout_character.get_selected_skin()->get_vertex_property<Point>("v:undef_point");

    if (!undefmesh_prop)
    {
        std::cerr << "correct_teeth: [ERROR] No undeformed mesh vertices found." << std::endl;
        return false;
    }


    // update teeth -------------------------------------------------------

    std::vector<unsigned int> mouth_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mouth_vec, fn_sel_mouth_region))
    {
        std::cerr << "correct_teeth: [ERROR] Could not load selection from file: \"" << fn_sel_mouth_region << "\"." << std::endl;
        return false;
    }

    std::vector< Point > src_points;
    std::vector< Point > tar_points;
    for ( unsigned int i = 0; i < mouth_vec.size(); ++i)
    {
        Surface_mesh::Vertex v(mouth_vec[i]);
        src_points.push_back( undefmesh_prop[v] );
        tar_points.push_back( base_mesh->position(v) );
    }

    // similarity transformation
    //        const Mat4f M_teeth = graphene::geometry::registration(src_points, tar_points, graphene::geometry::CONFORMAL_REGISTRATION);
    const Mat4f M_teeth = graphene::geometry::registration(src_points, tar_points, graphene::geometry::R_TRANS_SCALE_ANISO_XYZ);
    transform_mesh(*teeth_up_mesh, M_teeth);
    transform_mesh(*teeth_down_mesh, M_teeth);

    // update teeth blendshape targets
    graphene::character::Blendshapes& blendshapes_teeth_down                         = *inout_character.blendshapes()[1];
    std::vector<graphene::character::Blendshapes::Target_shape*>& targets_teeth_down = blendshapes_teeth_down.targets();
    for (unsigned int i = 0; i < targets_teeth_down.size(); ++i)
    {
        transform_mesh(*targets_teeth_down[i], M_teeth);
    }

    return true;
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

