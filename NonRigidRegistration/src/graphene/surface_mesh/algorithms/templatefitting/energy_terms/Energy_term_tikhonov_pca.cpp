//== INCLUDES =================================================================


#include "Energy_term_tikhonov_pca.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_tikhonov_pca::
Energy_term_tikhonov_pca(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Tikhonov PCA")
{
}


//-----------------------------------------------------------------------------


Energy_term_tikhonov_pca::
~Energy_term_tikhonov_pca()
{}


//-----------------------------------------------------------------------------


double
Energy_term_tikhonov_pca::
evaluate()
{
    const PCA& pca = energy_term_data_.pca;

    double error_reg_pca_tikhonov = 0.0;

    for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
    {
        double u = pca.d_pca_(dcount);
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * u * u;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * std::pow( u, robustness_parameters_.parameter );
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * ( huber_thr*u - huber_thr*huber_thr/2.0 );
            }
            else
            {
                error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * ( u*u/2.0 );
            }
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            double scale_parameter = robustness_parameters_.parameter;
//                error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
              error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ); // VERSION 3
        }
        else // default to L2
        {
            error_reg_pca_tikhonov += pca.inv_eigenvalues_pca_(dcount, dcount) * u * u;
        }
    }
    error_reg_pca_tikhonov *= (weight_ / pca.dim_pca_model_);

    return error_reg_pca_tikhonov;
}


//-----------------------------------------------------------------------------


void
Energy_term_tikhonov_pca::
add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r)
{
    //std::cout << "[INFO] Energy_term_tikhonov_pca::add_rows_small_lse(): weight_: " << weight_ << std::endl;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_tikhonov_pca::add_rows_small_lgs(...)'" << std::endl;
#endif

    // TODO currently not supported
    std::cerr << "Energy_term_tikhonov_pca::add_rows_small_lse(...) currently not supported" << std::endl;
    exit(1);

//    const double reg_weight = ...
//    const unsigned int offset_nrd_w_pca = ...
//    graphene::utility::PCA& pca = ...
}


//-----------------------------------------------------------------------------


void
Energy_term_tikhonov_pca::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    //std::cout << "[INFO] Energy_term_tikhonov_pca::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const double reg_weight = weight_;
    const PCA& pca = energy_term_data_.pca;



#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_tikhonov_pca::add_rows_big_lgs(...)'" << std::endl;
#endif

    const double weight_reg = sqrt(reg_weight / pca.dim_pca_model_);
    for (unsigned int i = 0; i < pca.dim_pca_model_; ++i)
    {
        // right hand side
        b(r) = 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_nrd_w_pca + i, weight_reg * sqrt( pca.inv_eigenvalues_pca_(i, i) ) ) );
        ++r;
    }
}

//-----------------------------------------------------------------------------

unsigned int
Energy_term_tikhonov_pca::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_tikhonov_pca::n_rows_big_sle()" << std::endl;

    return energy_term_data_.pca.dim_pca_model_;
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
