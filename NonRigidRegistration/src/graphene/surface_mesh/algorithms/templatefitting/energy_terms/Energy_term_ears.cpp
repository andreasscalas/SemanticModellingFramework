//== INCLUDES =================================================================


#include "Energy_term_ears.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_ears::
Energy_term_ears(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Ears")
{}


//-----------------------------------------------------------------------------


Energy_term_ears::
~Energy_term_ears()
{}


//-----------------------------------------------------------------------------


double
Energy_term_ears::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Point_set*        point_set = energy_term_data_.point_set;


    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > ears_lm_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ears_lm");
    const std::vector<Surface_mesh::Vertex>& ears_lm  = ears_lm_prop[0];

    const std::vector<unsigned int>& ears_targets_idx = point_set->lm_ears_;


    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_match_ears = 0.0;
    const unsigned int L = ears_targets_idx.size();
    for (unsigned int i = 0; i < L; ++i)
    {
        double u = norm( point_set->points_[ears_targets_idx[i]] - template_points[ears_lm[i]] );
        error_match_ears += u * u;
    }
    error_match_ears *= (weight_ / L);

    return error_match_ears;
}


//-----------------------------------------------------------------------------


void
Energy_term_ears::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{

    //std::cout << "[INFO] Energy_term_ears::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Point_set*        point_set = energy_term_data_.point_set;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > ears_lm_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ears_lm");
    const std::vector<Surface_mesh::Vertex>& ears_lm  = ears_lm_prop[0];

    const std::vector<unsigned int>& ears_targets_idx = point_set->lm_ears_;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = ears_targets_idx.size();

    const double ears_weight = sqrt(weight_ / L);
    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx = ears_lm[i].idx();
        const Point t = point_set->points_[ears_targets_idx[i]];

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = ears_weight * t[j];
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx, ears_weight) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_ears::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_ears::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Point_set*        point_set = energy_term_data_.point_set;

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > ears_lm_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ears_lm");
    const std::vector<Surface_mesh::Vertex>& ears_lm  = ears_lm_prop[0];

    const std::vector<unsigned int>& ears_targets_idx = point_set->lm_ears_;


    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int L = ears_targets_idx.size();

    const double weight_match_ears = sqrt(weight_ / L);

    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx = ears_lm[i].idx();
        const Point t = point_set->points_[ears_targets_idx[i]];

        // right hand side
        b(r) = weight_match_ears * t[0];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx, weight_match_ears) );
        ++r;

        // right hand side
        b(r) = weight_match_ears * t[1];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 1, weight_match_ears) );
        ++r;

        // right hand side
        b(r) = weight_match_ears * t[2];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 2, weight_match_ears) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_ears::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_ears::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Point_set*        point_set = energy_term_data_.point_set;

    const std::vector<unsigned int>& ears_targets_idx = point_set->lm_ears_;

    const unsigned int L = ears_targets_idx.size();

    return L;
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_ears::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_ears::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Point_set*        point_set = energy_term_data_.point_set;

    const std::vector<unsigned int>& ears_targets_idx = point_set->lm_ears_;

    const unsigned int L = ears_targets_idx.size();

    return 3*L;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
