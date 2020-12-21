//== INCLUDES =================================================================


#include "Energy_term_keep_vertices.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_keep_vertices::
Energy_term_keep_vertices(const Energy_term_data& energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Keep Vertices")
{
}


//-----------------------------------------------------------------------------


Energy_term_keep_vertices::
~Energy_term_keep_vertices()
{}


//-----------------------------------------------------------------------------


double
Energy_term_keep_vertices::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points      = template_mesh->get_vertex_property<Point>("v:point");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto template_keep_sel    = template_mesh->get_vertex_property<bool>("v:keep");

    double error_keep_vertices = 0.0;
//    int n = template_mesh->n_vertices();
    unsigned int n = 0;
    for ( auto v : template_mesh->vertices() )
    {
        if (template_keep_sel[v])
        {
            error_keep_vertices += sqrnorm( template_points[v] - template_restpose_points[v] );

            ++n;
        }
    }
    error_keep_vertices *= (weight_ / n);

    return error_keep_vertices;
}



//-----------------------------------------------------------------------------


void
Energy_term_keep_vertices::
add_rows_small_lse(
        std::vector<Tripl> &coeffs,
        Eigen::MatrixXd &B,
        unsigned int &r )
{
    //std::cout << "[INFO] Energy_term_keep_vertices::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto template_keep_sel    = template_mesh->get_vertex_property<bool>("v:keep");

    unsigned int n = 0;
    for ( auto v : template_mesh->vertices() ) // TODO nach außen verlagern
    {
        if (template_keep_sel[v])
        {
            ++n;
        }
    }

    const double keep_vertices_weight = sqrt(weight_ / n);
    for ( auto v : template_mesh->vertices() )
    {
        if (template_keep_sel[v])
        {        
            // right hand side
            for (unsigned int j = 0; j < 3; ++j)
            {
                B(r, j) = keep_vertices_weight * template_restpose_points[v][j];
            }

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + v.idx(), keep_vertices_weight) );
            ++r;
        }
    }
}



//-----------------------------------------------------------------------------


void
Energy_term_keep_vertices::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_keep_vertices::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto template_keep_sel    = template_mesh->get_vertex_property<bool>("v:keep");

    unsigned int n = 0;
    for ( auto v : template_mesh->vertices() ) // TODO nach außen verlagern
    {
        if (template_keep_sel[v])
        {
            ++n;
        }
    }

    const double keep_vertices_weight = sqrt(weight_ / n);
    for ( auto v : template_mesh->vertices() )
    {
        if (template_keep_sel[v])
        {        
            int idx = v.idx();

            // right hand side
            b(r) = keep_vertices_weight * template_restpose_points[v][0];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx, keep_vertices_weight) );
            ++r;

            // right hand side
            b(r) = keep_vertices_weight * template_restpose_points[v][1];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 1, keep_vertices_weight) );
            ++r;

            // right hand side
            b(r) = keep_vertices_weight * template_restpose_points[v][2];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 2, keep_vertices_weight) );
            ++r;
        }
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_keep_vertices::
n_rows_small_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_keep_sel = template_mesh->get_vertex_property<bool>("v:keep");

    if (!template_keep_sel)
    {
        std::cerr << "Energy_term_keep_vertices::n_rows_big_sle: [ERROR] Mesh has no property \"v:keep\"." << std::endl;
        return 0;
    }

    unsigned int n = 0;
    for ( auto v : template_mesh->vertices() ) // TODO nach außen verlagern
    {
        if (template_keep_sel[v])
        {
            ++n;
        }
    }

    return n;
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_keep_vertices::
n_rows_big_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_keep_sel = template_mesh->get_vertex_property<bool>("v:keep");

    if (!template_keep_sel)
    {
        std::cerr << "Energy_term_keep_vertices::n_rows_big_sle: [ERROR] Mesh has no property \"v:keep\"." << std::endl;
        return 0;
    }

    unsigned int n = 0;
    for ( auto v : template_mesh->vertices() ) // TODO nach außen verlagern
    {
        if (template_keep_sel[v])
        {
            ++n;
        }
    }

    return 3*n;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
