//== INCLUDES =================================================================


#include "Energy_term_eyes_closed.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_eyes_closed::
Energy_term_eyes_closed(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Eyes-Closed")
{}


//-----------------------------------------------------------------------------


Energy_term_eyes_closed::
~Energy_term_eyes_closed()
{}


//-----------------------------------------------------------------------------


double
Energy_term_eyes_closed::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > eyes_closed_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:eyes_closed");
    const std::vector<Surface_mesh::Vertex>& eyes_closed  = eyes_closed_prop[0];

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_match_eyes_closed = 0.0;
    const unsigned int L = eyes_closed.size() / 2;
    for (unsigned int i = 0; i < L; ++i)
    {
        double u = norm( template_points[eyes_closed[2*i]] - template_points[eyes_closed[2*i+1]] );
        error_match_eyes_closed += u * u;
    }
    error_match_eyes_closed *= (weight_ / L);

    return error_match_eyes_closed;
}


//-----------------------------------------------------------------------------


void
Energy_term_eyes_closed::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{

    //std::cout << "[INFO] Energy_term_eyes_closed::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > eyes_closed_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:eyes_closed");
    const std::vector<Surface_mesh::Vertex>& eyes_closed  = eyes_closed_prop[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = eyes_closed.size() / 2;

    const double eyes_closed_weight = sqrt(weight_ / L);
    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx1 = eyes_closed[2*i  ].idx();
        const int idx2 = eyes_closed[2*i+1].idx();

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = eyes_closed_weight * 0.0;
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx1,  eyes_closed_weight) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx2, -eyes_closed_weight) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_eyes_closed::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_eyes_closed::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > eyes_closed_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:eyes_closed");
    const std::vector<Surface_mesh::Vertex>& eyes_closed  = eyes_closed_prop[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = eyes_closed.size() / 2;

    const double weight_match_eyes_closed = sqrt(weight_ / L);

    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx1 = eyes_closed[2*i  ].idx();
        const int idx2 = eyes_closed[2*i+1].idx();

        // right hand side
        b(r) = weight_match_eyes_closed * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1,  weight_match_eyes_closed) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2, -weight_match_eyes_closed) );
        ++r;

        // right hand side
        b(r) = weight_match_eyes_closed * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1 + 1,  weight_match_eyes_closed) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2 + 1, -weight_match_eyes_closed) );
        ++r;

        // right hand side
        b(r) = weight_match_eyes_closed * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1 + 2,  weight_match_eyes_closed) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2 + 2, -weight_match_eyes_closed) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_eyes_closed::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_eyes_closed::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > eyes_closed_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:eyes_closed");
    const std::vector<Surface_mesh::Vertex>& eyes_closed  = eyes_closed_prop[0];

    const unsigned int L = eyes_closed.size() / 2;

    return L;
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_eyes_closed::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_eyes_closed::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > eyes_closed_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:eyes_closed");
    const std::vector<Surface_mesh::Vertex>& eyes_closed  = eyes_closed_prop[0];

    const unsigned int L = eyes_closed.size() / 2;

    return 3*L;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
