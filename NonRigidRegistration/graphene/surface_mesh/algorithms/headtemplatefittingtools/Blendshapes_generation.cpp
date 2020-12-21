//== INCLUDES ===================================================================


#include "Blendshapes_generation.h"

#include <graphene/surface_mesh/eigen_algorithms/deformation_transfer/Deformation_transfer_botsch.h>
#include <graphene/surface_mesh/algorithms/nonlinear_discrete_shell/Nonlinear_discrete_shell.h>

#include <fstream>


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================


Blendshapes_generation::
Blendshapes_generation()
{}


//-----------------------------------------------------------------------------

bool
Blendshapes_generation::
transfer_blendshapes(character::Character &ch, const std::string &base_mesh, const std::string& fn_nonlin_selection, bool non_linear)
{
    const std::vector<character::Blendshapes*>& blendshape_sets = ch.blendshapes();

    character::Blendshapes* selected_bs_set = nullptr;

    for (size_t i=0; i < blendshape_sets.size(); ++i)
    {
        character::Blendshapes* bs = blendshape_sets[i];
        Surface_mesh::Mesh_property<std::string> mid = bs->base()->get_mesh_property<std::string>("m:id");
        if (mid && mid[0] == base_mesh)
        {
            selected_bs_set = bs;
            break;
        }
    }

    if (selected_bs_set == nullptr)
    {
        std::cerr << "Blendshapes_generation::transfer_blendshapes: [ERROR] Cannot transfer blendshapes. Did not find base mesh with blendshapes: \"" << base_mesh << "\"" << std::endl;
        return false;
    }


    Surface_mesh::Vertex_property<Point> undef_source_prop = selected_bs_set->base()->get_vertex_property<Point>("v:undef_point");
    if (!undef_source_prop)
    {
        std::cerr << "Blendshapes_generation::transfer_blendshapes: [ERROR] No undeformed data." << std::endl;
        return false;
    }

    //create new undeformed mesh
    Surface_mesh mesh_source_undeformed = *selected_bs_set->base();
    Surface_mesh::Vertex_property<Point> vp = mesh_source_undeformed.get_vertex_property<Point>("v:point");
    vp.vector() = undef_source_prop.vector();

    if (non_linear)
    {
        by_discrete_shell(mesh_source_undeformed,
                          *selected_bs_set->base(),
                          *selected_bs_set,
                          fn_nonlin_selection
                    );
    }
    else
    {
        by_deformation_transfer(mesh_source_undeformed, *selected_bs_set->base(), *selected_bs_set);
    }

    return true;
}

//-----------------------------------------------------------------------------


bool
Blendshapes_generation::
by_deformation_transfer(graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                        graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                        graphene::character::Blendshapes& blendshapes_body)
{


    // update targets
    std::vector<graphene::character::Blendshapes::Target_shape*>& targets_body = blendshapes_body.targets();
    for (unsigned int bs = 0; bs < targets_body.size(); ++bs)
    {
        // generate new blendshape by deformation transfer

        // we need the fitted neutral shape (target undeformed)
        // we need the undeformed source shape (source undeformed)
        // for every new blendshape, we need the old one (source deformed)
        graphene::surface_mesh::Surface_mesh& mesh_source_deformed = *targets_body[bs];

        // deformation transfer
        graphene::surface_mesh::Surface_mesh mesh_target_deformed = mesh_target_undeformed;

        graphene::surface_mesh::Deformation_transfer_botsch deftrans;
        if (!deftrans.create_deformed_targets(mesh_source_undeformed,
                                              mesh_source_deformed,
                                              mesh_target_deformed))
        {
            return false;
        }

        // update blendshape target
        for (auto v : targets_body[bs]->vertices())
        {
            targets_body[bs]->position(v) = mesh_target_deformed.position(v);
        }
        targets_body[bs]->update_face_normals();
        targets_body[bs]->update_vertex_normals();
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Blendshapes_generation::
by_discrete_shell(graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                  graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                  graphene::character::Blendshapes& blendshapes_body,
                  const std::string& filename_fixed_nonlindeftrans)
{
    // update targets
    std::vector<graphene::character::Blendshapes::Target_shape*>& targets_body = blendshapes_body.targets();
    for (unsigned int bs = 0; bs < targets_body.size(); ++bs)
    {
        // generate new blendshape by deformation transfer

        // we need the fitted neutral shape (target undeformed)
        // we need the undeformed source shape (source undeformed)
        // for every new blendshape, we need the old one (source deformed)
        graphene::surface_mesh::Surface_mesh& mesh_source_deformed = *targets_body[bs];

        // deformation transfer
        graphene::surface_mesh::Surface_mesh mesh_target_deformed = mesh_target_undeformed;

        auto region = mesh_target_deformed.vertex_property<Region_type>("v:region"); // TODO fix more points to speed up
        for (auto v : mesh_target_deformed.vertices())
        {
            region[v] = deformable;
        }
        // Surface_mesh::Vertex v0(11348);
        // region[v0] = fixed;
        // Surface_mesh::Vertex v1(11295);
        // region[v1] = fixed;
        // Surface_mesh::Vertex v2(11296);
        // region[v2] = fixed;
        // Surface_mesh::Vertex v3(689);
        // region[v3] = fixed;
        // Surface_mesh::Vertex v4(690);
        // region[v4] = fixed;
        // Surface_mesh::Vertex v5(738);
        // region[v5] = fixed;

        std::ifstream ifs;
        ifs.open(filename_fixed_nonlindeftrans.c_str());
        if (!ifs)
        {
            std::cerr << "[ERROR] Can't open selection of body." << std::endl;
            return false;
        }
        unsigned int idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;

            Surface_mesh::Vertex v(idx);
            region[v] = fixed;
        }


        graphene::surface_mesh::Nonlinear_discrete_shell nonlin_ds;

        if (!nonlin_ds.set_base_mesh(mesh_target_deformed))
        {
            std::cerr << "[ERROR] Can't set base mesh." << std::endl;
            return false;
        }

        if (!nonlin_ds.set_stiffness(10000, 10))
        {
            std::cerr << "[ERROR] Can't set stiffness." << std::endl;
            return false;
        }

        if (!nonlin_ds.deformation_transfer(mesh_source_undeformed, mesh_source_deformed))
        {
            std::cerr << "[ERROR] Can't transfer deformation." << std::endl;
            return false;
        }


        // update blendshape target
        for (auto v : targets_body[bs]->vertices())
        {
            targets_body[bs]->position(v) = mesh_target_deformed.position(v);
        }
        targets_body[bs]->update_face_normals();
        targets_body[bs]->update_vertex_normals();
    }

    return true;    
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
