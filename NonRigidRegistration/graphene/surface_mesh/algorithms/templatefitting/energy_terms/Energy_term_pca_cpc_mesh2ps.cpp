//== INCLUDES =================================================================


#include "Energy_term_pca_cpc_mesh2ps.h"

#include "../settings.h"

#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_pca_cpc_mesh2ps::
Energy_term_pca_cpc_mesh2ps(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "PCA CPC Mesh-to-PointSet")
{
}


//-----------------------------------------------------------------------------


Energy_term_pca_cpc_mesh2ps::
~Energy_term_pca_cpc_mesh2ps()
{}


//-----------------------------------------------------------------------------

void Energy_term_pca_cpc_mesh2ps::pre_minimize_action()
{
    //skin_correspondences_inverse_mesh2ps(energy_term_data_.template_mesh, energy_term_data_.correspondences, skinned_correspondences);
    skin_correspondences(energy_term_data_.template_mesh, energy_term_data_.correspondences, skinned_correspondences, false);
}

//-----------------------------------------------------------------------------


double
Energy_term_pca_cpc_mesh2ps::
evaluate()
{
    const std::vector<Correspondence>& correspondences = energy_term_data_.correspondences;
    const PCA& pca = energy_term_data_.pca;

    double error_reg_pca_cpc_mesh2ps = 0.0;

    const unsigned int C = correspondences.size();

    for (unsigned int i = 0; i < C; ++i)
    {
        int idx = correspondences[i].on_template_vh.idx();

        double mi_x = pca.m_pca_(3*idx    );
        double mi_y = pca.m_pca_(3*idx + 1);
        double mi_z = pca.m_pca_(3*idx + 2);
        const Point  mi(mi_x, mi_y, mi_z); // TODO EVTL. SCHNELLER ?!

        Point  Pid(0.0, 0.0, 0.0);
        for (unsigned int j = 0; j < 3; ++j)
        {
            for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
            {
                Pid[j] += pca.P_pca_(3*idx + j, dcount) * pca.d_pca_(dcount);
            }
        }

        double u = norm( Pid + mi - correspondences[i].on_ps );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_reg_pca_cpc_mesh2ps += u * u;
        }
        else
        {
            std::cerr << "[ERROR] in 'Energy_term_pca_cpc_mesh2ps::evaluate()' robust function not supported! Aborting..." << std::endl;
            exit(1);
        }
    }
    error_reg_pca_cpc_mesh2ps *= (weight_ / C);

    return error_reg_pca_cpc_mesh2ps;
}


//-----------------------------------------------------------------------------


void
Energy_term_pca_cpc_mesh2ps::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    std::cerr << "[INFO] in: Energy_term_pca_cpc_mesh2ps::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    //choose either the skinnend correspondences or the "normal" ones based on whether skinned correspondences exist or not
    const std::vector<Correspondence>& correspondences = skinned_correspondences.empty() ? energy_term_data_.correspondences : skinned_correspondences;
    const unsigned int offset_nrd_w_pca = column_offsets.offset_nrd_w_pca;
    const PCA& pca = energy_term_data_.pca;

    const unsigned int C = correspondences.size();

    const double weight_model = sqrt(weight_ / C);
    //std::cout << "[INFO] Energy_term_pca_cpc_mesh2ps::add_rows_big_lse(): weight_model: " << weight_model << std::endl;
    for (unsigned int i = 0; i < C; ++i)
    {
        if (correspondences[i].constr_dir == graphene::surface_mesh::corresp_dir_ps2mesh ||
            correspondences[i].constr_dir == graphene::surface_mesh::corresp_dir_mesh2ps_and_ps2mesh ) // TODO EVTL. WOANDERS HIN
        {
            std::cerr << "[ERROR] in 'Energy_term_pca_cpc_mesh2ps::add_rows_big_lse()' bad exception! Aborting..." << std::endl;
            exit(1);
        }

        int idx = correspondences[i].on_template_vh.idx();

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[0] - pca.m_pca_(3*idx + 0) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, pca.P_pca_(3*idx, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[1] - pca.m_pca_(3*idx + 1) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, pca.P_pca_(3*idx + 1, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[2] - pca.m_pca_(3*idx + 2) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, pca.P_pca_(3*idx + 2, dcount) * weight_model) ); // d's
        }
        ++r;
    }
}

//-----------------------------------------------------------------------------



unsigned int
Energy_term_pca_cpc_mesh2ps::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_pca_cpc_mesh2ps::n_rows_big_sle()" << std::endl;

    return (unsigned int)3*energy_term_data_.correspondences.size();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
