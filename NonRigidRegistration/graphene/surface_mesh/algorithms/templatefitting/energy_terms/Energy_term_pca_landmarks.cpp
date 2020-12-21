//== INCLUDES =================================================================


#include "Energy_term_pca_landmarks.h"
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_pca_landmarks::
Energy_term_pca_landmarks(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "PCA Landmarks")
{
}


//-----------------------------------------------------------------------------


Energy_term_pca_landmarks::
~Energy_term_pca_landmarks()
{}


//-----------------------------------------------------------------------------


void
Energy_term_pca_landmarks::
pre_minimize_action()
{
    //skin_pointset_landmarks_inverse(energy_term_data_.template_mesh, energy_term_data_.point_set, energy_term_data_.landmarks_manager, skinned_landmarks_pointset_);
    skin_pointset_landmarks(energy_term_data_.template_mesh, energy_term_data_.point_set, energy_term_data_.landmarks_manager, skinned_landmarks_pointset_, false);
}


//-----------------------------------------------------------------------------


double
Energy_term_pca_landmarks::
evaluate()
{
    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    const PCA& pca = energy_term_data_.pca;
    const Point_set* point_set = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    if (!landmarks_manager.check_dimension(point_set, template_mesh))
    {
        return DBL_MAX;
    }

    std::vector<Surface_mesh::Vertex> tm_landmarks;
    landmarks_manager.get_indices_template(template_mesh, tm_landmarks);

    double error_reg_pca_ff = 0.0;

    std::vector<Point> lm_ps;
    landmarks_manager.get_landmarks_point_set_stdvec(point_set, lm_ps);
    const unsigned int           L(lm_ps.size());

    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx = tm_landmarks[i].idx();

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

        double u = norm( Pid + mi - lm_ps[i] );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_reg_pca_ff += u * u;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_reg_pca_ff += std::pow( u, robustness_parameters_.parameter);
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_reg_pca_ff += ( huber_thr*u - huber_thr*huber_thr/2.0 );
            }
            else
            {
                error_reg_pca_ff += ( u*u/2.0 );
            }
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            double scale_parameter = robustness_parameters_.parameter;
//                error_reg_pca_ff += ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
              error_reg_pca_ff += ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3
        }
        else // default to L2
        {
            error_reg_pca_ff += u * u;
        }
    }
    error_reg_pca_ff *= (weight_ / L);

    return error_reg_pca_ff;
}


//-----------------------------------------------------------------------------


void
Energy_term_pca_landmarks::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r )
{
    //std::cout << "[INFO] Energy_term_pca_landmarks::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_nrd_w_pca = column_offsets.offset_nrd_w_pca;
    const PCA& pca = energy_term_data_.pca;
    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    const geometry::Point_set* point_set = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;


    if (!landmarks_manager.check_dimension(point_set, template_mesh))
    {
        std::cerr << "Energy_term_pca_landmarks::add_rows_big_lse(): [ERROR] Number of landmarks do not match." << std::endl;
        return;
    }

    std::vector<Surface_mesh::Vertex> tm_landmarks;
    landmarks_manager.get_indices_template(template_mesh, tm_landmarks);
    std::vector<Point> lm_ps;
    landmarks_manager.get_landmarks_point_set_stdvec(point_set, lm_ps);//std::cout << "[INFO] Energy_term_nonlin_aniso_laplace::n_rows_big_sle()" << std::endl;
    const unsigned int           L(lm_ps.size());


    //choose either the skinnend landmarks or the "normal" ones based on whether skinned landmarks exist or not
    const std::vector<Point>& landmarks_pointset = skinned_landmarks_pointset_.empty() ? lm_ps : skinned_landmarks_pointset_;

    const double weight_model = sqrt(weight_ / L);
    //std::cout << "[INFO] Energy_term_pca_landmarks::add_rows_big_lse(): weight_model: " << weight_model << std::endl;
    for (unsigned int i = 0; i < L; ++i)
    {
        int idx = tm_landmarks[i].idx();

        // right hand side
        b(r) = weight_model * ( landmarks_pointset[i][0] - pca.m_pca_(3*idx + 0) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, pca.P_pca_(3*idx, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( landmarks_pointset[i][1] - pca.m_pca_(3*idx + 1) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, pca.P_pca_(3*idx + 1, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( landmarks_pointset[i][2] - pca.m_pca_(3*idx + 2) );

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
Energy_term_pca_landmarks::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_pca_landmarks::n_rows_big_sle()" << std::endl;

    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    return (unsigned int) 3*landmarks_manager.n_landmarks_pointset(energy_term_data_.point_set);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
