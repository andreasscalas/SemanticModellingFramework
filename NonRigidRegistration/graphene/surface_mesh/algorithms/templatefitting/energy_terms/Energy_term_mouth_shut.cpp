//== INCLUDES =================================================================


#include "Energy_term_mouth_shut.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_mouth_shut::
Energy_term_mouth_shut(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Mouth-Shut")
{}


//-----------------------------------------------------------------------------


Energy_term_mouth_shut::
~Energy_term_mouth_shut()
{}


//-----------------------------------------------------------------------------


double
Energy_term_mouth_shut::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    const std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_match_mouth_shut = 0.0;
    const unsigned int L = mouth_shut.size() / 2;
    for (unsigned int i = 0; i < L; ++i)
    {
        double u = norm( template_points[mouth_shut[2*i]] - template_points[mouth_shut[2*i+1]] );
        error_match_mouth_shut += u * u;
    }
    error_match_mouth_shut *= (weight_ / L);

    return error_match_mouth_shut;
}


//-----------------------------------------------------------------------------


void
Energy_term_mouth_shut::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{

    //std::cout << "[INFO] Energy_term_mouth_shut::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    const std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = mouth_shut.size() / 2;

    const double mouth_shut_weight = sqrt(weight_ / L);
    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx1 = mouth_shut[2*i  ].idx();
        const int idx2 = mouth_shut[2*i+1].idx();

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = mouth_shut_weight * 0.0;
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx1,  mouth_shut_weight) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx2, -mouth_shut_weight) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_mouth_shut::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_mouth_shut::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    const std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = mouth_shut.size() / 2;

    const double weight_match_mouth_shut = sqrt(weight_ / L);

    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx1 = mouth_shut[2*i  ].idx();
        const int idx2 = mouth_shut[2*i+1].idx();

        // right hand side
        b(r) = weight_match_mouth_shut * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1,  weight_match_mouth_shut) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2, -weight_match_mouth_shut) );
        ++r;

        // right hand side
        b(r) = weight_match_mouth_shut * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1 + 1,  weight_match_mouth_shut) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2 + 1, -weight_match_mouth_shut) );
        ++r;

        // right hand side
        b(r) = weight_match_mouth_shut * 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx1 + 2,  weight_match_mouth_shut) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx2 + 2, -weight_match_mouth_shut) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_mouth_shut::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_mouth_shut::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    const std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    const unsigned int L = mouth_shut.size() / 2;

    return L;
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_mouth_shut::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_mouth_shut::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    const std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    const unsigned int L = mouth_shut.size() / 2;

    return 3*L;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
