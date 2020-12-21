//== INCLUDES ===================================================================


#include "Deformation_transfer.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/diffgeo.h>
#include <graphene/geometry/Matrix3x3.h>

#include <cfloat>
#include <fstream>


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================


Deformation_transfer::
Deformation_transfer()
{
   solver_ = 0;
   A_ = 0;
   factorization_state = false;
}


//-----------------------------------------------------------------------------


Deformation_transfer::
~Deformation_transfer()
{
   delete A_;
   delete solver_;
}

//-----------------------------------------------------------------------------


void
Deformation_transfer::
create_deformed_targets(const Deformation_transfer_data& data)
{
    // read neutral source mesh
    Surface_mesh src_neutral_mesh;
    src_neutral_mesh.read(data.source_undeformed.c_str());
    if (!src_neutral_mesh.is_triangle_mesh())
    {
        src_neutral_mesh.triangulate();
    }

    // read neutral target mesh
    Surface_mesh tar_neutral_mesh;
    tar_neutral_mesh.read(data.target_undeformed.c_str());
    if (!tar_neutral_mesh.is_triangle_mesh())
    {
        tar_neutral_mesh.triangulate();
    }

    setup_locked_vert(src_neutral_mesh, data.source_deformed, tar_neutral_mesh);

    unsigned int mat_size = 0;
    prefactorize(tar_neutral_mesh, mat_size);

    //apply deformation transfer
    std::set<std::string>::iterator it_file = data.source_deformed.begin();
    while (it_file != data.source_deformed.end())
    {
        // read current defomed source
        Surface_mesh src_deformed_mesh;
        src_deformed_mesh.read((*it_file).c_str());
        if (!src_deformed_mesh.is_triangle_mesh())
        {
            src_deformed_mesh.triangulate();
        }

        // generate target
        Surface_mesh tar_deformed_mesh(tar_neutral_mesh);
        create_deformed_target(src_neutral_mesh,
                               src_deformed_mesh,
                               tar_neutral_mesh,
                               tar_deformed_mesh,
                               mat_size);

        // generate name for current target expression
        std::string filename_out = (*it_file).c_str();
        int pos1 = filename_out.find_last_of("/");
        int pos2 = filename_out.find_last_of("\\");
        pos1 = std::max(pos1, pos2);
        filename_out = filename_out.substr(pos1+1);
        filename_out.insert(0,"/new_");
        filename_out.insert(0, data.location_target_deformed);

        // write current target expression
        tar_deformed_mesh.write(filename_out);
        std::cout << "file saved: " << filename_out << std::endl;
        it_file++;
    }
}


//-----------------------------------------------------------------------------


bool
Deformation_transfer::
create_deformed_targets(Surface_mesh& source_undeformed,
                        Surface_mesh& source_deformed,
                        Surface_mesh& target_undeformed)
{
    // triangulate
    if (!source_undeformed.is_triangle_mesh())
    {
        source_undeformed.triangulate();
    }
    if (!source_deformed.is_triangle_mesh())
    {
        source_deformed.triangulate();
    }
    if (!target_undeformed.is_triangle_mesh())
    {
        target_undeformed.triangulate();
    }

    setup_locked_vert(source_undeformed, source_deformed, target_undeformed);

    unsigned int mat_size = 0;
    prefactorize(target_undeformed, mat_size);

    //apply deformation transfer

    // generate target
    Surface_mesh tar_deformed_mesh(target_undeformed);
    if (!create_deformed_target(source_undeformed,
                                source_deformed,
                                target_undeformed,
                                tar_deformed_mesh,
                                mat_size))
    {
        return false;
    }

    // overwrite target expression
    target_undeformed = tar_deformed_mesh;

    return true;
}


//-----------------------------------------------------------------------------


void
Deformation_transfer::
setup_locked_vert(Surface_mesh& src_neutral_mesh,
                  const std::set<std::string>& files_src_deformed,
                  Surface_mesh& tar_neutral_mesh)
{
    // find fixed vertices

    std::set<std::string>::iterator it_file = files_src_deformed.begin();
    while (it_file != files_src_deformed.end())
    {
        // read current defomed source
        Surface_mesh current_src_expression_mesh;
        current_src_expression_mesh.read((*it_file).c_str());
        if (!current_src_expression_mesh.is_triangle_mesh())
        {
            current_src_expression_mesh.triangulate();
        }

        // get delta for each vertex
        compute_deltas(src_neutral_mesh, current_src_expression_mesh);

        // close mesh
        current_src_expression_mesh.clear();
        it_file++;
    }

    // lock vertices with small delta
    lock_vertices_small_delta(src_neutral_mesh,
                              tar_neutral_mesh);

    auto locked   = tar_neutral_mesh.vertex_property<bool>("v:locked");
    auto selected = tar_neutral_mesh.vertex_property<bool>("v:selected");
    for (auto v : tar_neutral_mesh.vertices())
    {
        if (selected[v])
        {
            locked[v] = selected[v];
        }
    }
}


//-----------------------------------------------------------------------------


void
Deformation_transfer::
setup_locked_vert(Surface_mesh& src_neutral_mesh,
                  Surface_mesh& src_expr_mesh,
                  Surface_mesh& tar_neutral_mesh)
{
    // get delta for each vertex
    compute_deltas(src_neutral_mesh, src_expr_mesh);

    // lock vertices with small delta
    lock_vertices_small_delta(src_neutral_mesh,
                              tar_neutral_mesh);

    auto locked   = tar_neutral_mesh.vertex_property<bool>("v:locked");
    auto selected = tar_neutral_mesh.vertex_property<bool>("v:selected");
    for (auto v : tar_neutral_mesh.vertices())
    {
        if (selected[v])
        {
            locked[v] = selected[v];
        }
    }
}


//-----------------------------------------------------------------------------


void
Deformation_transfer::
compute_deltas(Surface_mesh& src_neutral_mesh,
               Surface_mesh& src_expr_mesh)
{
    auto src_undef_pos = src_neutral_mesh.vertex_property<Point>("v:point");
    auto src_def_pos   = src_expr_mesh.vertex_property<Point>("v:point");

    // compute first absolut delta for all vertices
    auto sum_diff = src_neutral_mesh.get_vertex_property<double>("v:sum_diff");
    if (!sum_diff)
    {
        sum_diff = src_neutral_mesh.vertex_property<double>("v:sum_diff", 0.0);
    }

    // get delta for each vertex
    for (auto v : src_neutral_mesh.vertices())
    {
        sum_diff[v] += distance(src_undef_pos[v], src_def_pos[v]);
    }
}


//-----------------------------------------------------------------------------


void
Deformation_transfer::
lock_vertices_small_delta(Surface_mesh& src_neutral_mesh,
                          Surface_mesh& tar_neutral_mesh)
{
    auto sum_diff = src_neutral_mesh.get_vertex_property<double>("v:sum_diff");

    // lock vertices with small delta
    auto locked = tar_neutral_mesh.vertex_property<bool>("v:locked");
    for (auto v : tar_neutral_mesh.vertices())
    {
        locked[v] = false;
    }
    double min_dist = DBL_MAX;
    Surface_mesh::Vertex  vh;
    bool locked_any = false;
    for (auto v : src_neutral_mesh.vertices())
    {
        if (sum_diff[v] < min_dist)
        {
            min_dist = sum_diff[v];
            vh = v;
        }
        if (sum_diff[v] < 1e-6)
        {
            locked[v] = true;
            locked_any = true;
        }
    }
    if (!locked_any) // lock vertex with least movement if all vertices are moving
    {
        locked[vh] = true;
        std::cerr << "[DEBUG] all vertices are moving: lock vertex with least movement" << std::endl;
    }
    src_neutral_mesh.remove_vertex_property(sum_diff);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
