//=============================================================================
#ifndef GRAPHENE_BLENDSHAPES_GENERATION_HELPER_H
#define GRAPHENE_BLENDSHAPES_GENERATION_HELPER_H
//=============================================================================

//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/character/data_structure/Character.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class Blendshapes_generation
{
public: //---------------------------------------------------- public functions

    // constructor
    Blendshapes_generation();

    bool transfer_blendshapes(character::Character& ch, const std::string& base_mesh, const std::string& fn_nonlin_selection,  bool non_linear=false);

private:

    bool by_deformation_transfer(graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                                 graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                                 graphene::character::Blendshapes& blendshapes_body);

    bool by_discrete_shell(graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                           graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                           graphene::character::Blendshapes& blendshapes_body,
                           const std::string& filename_fixed_nonlindeftrans);

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_BLENDSHAPES_GENERATION_HELPER_H
//=============================================================================
