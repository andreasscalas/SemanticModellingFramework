//== INCLUDES =================================================================


#include "Energy.h"

#include "settings.h"

#include <graphene/geometry/Matrix4x4.h>
#include <graphene/utility/Stop_watch.h>
#include <graphene/surface_mesh/algorithms/pca/utility/my_helper.h>

#include "utility/my_helper.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {

using graphene::Mat4;


//== IMPLEMENTATION ===========================================================


Energy::
Energy() :
    energy_term_list_(energy_term_data_)
{
}


//-----------------------------------------------------------------------------


Energy::
~Energy()
{}


//-----------------------------------------------------------------------------


void
Energy::
clear()
{
    function_value_     = DBL_MAX;

    energy_term_list_.clear();
    energy_term_data_.clear();
}


//-----------------------------------------------------------------------------


bool
Energy::
minimize(const double  conv_val)
{
    bool small_sle_supported = false;
    bool third_iter_done     = false;
    bool linear_solve        = false;
    bool irls_solve          = false;
    bool A_is_set            = false;

    unsigned int n_columns;
    unsigned int n_rows;

    double error_total_before_third_loop = DBL_MAX;
    double error_total_after_third_loop  = DBL_MAX;
    double conv_val_rel2                 = 0.0;

    std::set<Solve_result> solve_results;
    PCA& pca = energy_term_data_.pca;

    const unsigned int nv = energy_term_data_.template_mesh->n_vertices();


    //update list of active energy terms
    energy_term_list_.update_active();


    //to what we are solving? -> ask energy list
    solve_results = energy_term_list_.solve_results();

    //determine if small sle is supported -> ask energy list
    small_sle_supported = energy_term_list_.is_small_sle_supported();
//    std::cout << "Energy::minimize(): [INFO] is a small sle supported?: "
//              << small_sle_supported << std::endl;

    //do we only have linear energies? -> ask energy list
    linear_solve = ! energy_term_list_.is_non_linear();
//    std::cout << "Energy::minimize(): [INFO] do we only have linear energies?: "
//              << linear_solve << std::endl;

    //do we need the "expensive" irls solver? -> ask energy list
    irls_solve = ! energy_term_list_.is_all_L2();
//    std::cout << "Energy::minimize(): [INFO] do we need the expensive irls solver?: "
//              << irls_solve << std::endl;

    //do we need pca? -> ask energy list
    if ( energy_term_list_.need_pca() )
    {
        // check P matrix
        if ( !check_dimension_PCA( pca, nv ) ) // stop if we want to solve to pca with wrong dimension
        {
            std::cerr << "Energy::minimize(): [ERROR] Wrong dimension of PCA model. Aborting..." << std::endl;
            return false;
        }
        pca.d_pca_.setZero();
    }


    // attention: apply pre minimize actions before computing number of rows
    energy_term_list_.apply_pre_minimize_action();


    n_columns = compute_offsets_and_return_number_of_columns(solve_results, nv, small_sle_supported, pca.dim_pca_model_);
    //std::cout << "Energy::minimize(): [INFO] number of columns: "
    //          << n_columns << std::endl;

    // determine number of rows
    if (small_sle_supported)
    {
        //std::cout << "Energy::minimize(): [INFO] a small sle is supported" << std::endl;
        n_rows = energy_term_list_.n_rows_small_sle();
    }
    else
    {
        //std::cout << "Energy::minimize(): [INFO] a small sle is NOT supported" << std::endl;
        n_rows = energy_term_list_.n_rows_big_sle();
    }

    //std::cout << "Energy::minimize(): [INFO] number of rows: "
    //          << n_rows << std::endl;


    SpMat A(n_rows, n_columns);
    std::vector<Tripl>  coeffs;
    unsigned int r = 0;

    if (! linear_solve)
    {
        error_total_before_third_loop = energy_term_list_.evaluate();
    }

    if (small_sle_supported)
    {
        Eigen::MatrixXd X(n_columns, 3);
        Eigen::MatrixXd B(n_rows, 3);
        unsigned int iter_third_loop = 0;
        while(!third_iter_done)
        {
            iter_third_loop++;
            //std::cout << "3rd loop: " << iter_third_loop << std::endl;

            coeffs.clear();
            r = 0;

            energy_term_list_.assemble_small_lse(coeffs, B, r);

            if (irls_solve)
            {
                if (!A_is_set)
                {
                    A.setFromTriplets(coeffs.begin(), coeffs.end());
                    A_is_set = true;
                }
                //std::cout << "Energy::minimize(): [INFO] solve by IRLS-solver." << std::endl;
                if (! lls_solver_.solve_irls(A, B, energy_term_list_.active_terms(), X))
                {
                    std::cerr << "Energy::minimize(): [ERROR] IRLS-Solver failed. Aborting..." << std::endl;
                    return false;
                }
            }
            else
            {
                if (!A_is_set)
                {
                    A.setFromTriplets(coeffs.begin(), coeffs.end());
                    A_is_set = true;

                    if (! lls_solver_.compute(A))
                    {
                        std::cerr << "Energy::minimize(): [ERROR] Factorization failed (small SLE). Aborting..." << std::endl;
                        return false;
                    }
                }
                //std::cout << "Energy::minimize(): [INFO] solve by LLS-solver." << std::endl;
                if (! lls_solver_.solve(B, X))
                {
                    std::cerr << "Energy::minimize(): [ERROR] Solve failed. Aborting..." << std::endl;
                    return false;
                }
            }

            apply_solution(solve_results, X);

            energy_term_list_.apply_post_processing();

            if (linear_solve) // if linear -> done
            {
                function_value_ = energy_term_list_.evaluate();
                break;
            }
            else // if non-linear -> check termination criteria
            {
                function_value_ = error_total_after_third_loop = energy_term_list_.evaluate();
                conv_val_rel2 = std::abs(error_total_after_third_loop - error_total_before_third_loop) / error_total_before_third_loop;

                if (conv_val_rel2 < conv_val)
                    break;

                error_total_before_third_loop = error_total_after_third_loop;
            }
        }
    }
    else
    {
        Eigen::VectorXd x(n_columns);
        Eigen::VectorXd b(n_rows);
        unsigned int iter_third_loop = 0;
        while(!third_iter_done)
        {
            iter_third_loop++;
            //std::cout << "3rd loop: " << iter_third_loop << std::endl;

            coeffs.clear();
            r = 0;

            energy_term_list_.assemble_big_lse(coeffs, b, r);

            if (irls_solve)
            {
                if (!A_is_set)
                {
                    A.setFromTriplets(coeffs.begin(), coeffs.end());
                    A_is_set = true;
                }
                if (! lls_solver_.solve_irls(A, b, energy_term_list_.active_terms(), x))
                {
                    std::cerr << "Energy::minimize(): [ERROR] IRLS-Solver failed. Aborting..." << std::endl;
                    return false;
                }
            }
            else
            {
                if (!A_is_set)
                {
                    A.setFromTriplets(coeffs.begin(), coeffs.end());
                    A_is_set = true;

                    if (! lls_solver_.compute(A))
                    {
                        std::cerr << "Energy::minimize(): [ERROR] Factorization failed (big SLE).. Aborting..." << std::endl;
                        return false;
                    }
                }
                if (!lls_solver_.solve(b, x))
                {
                    std::cerr << "Energy::minimize(): [ERROR] Solve failed. Aborting..." << std::endl;
                    return false;
                }
            }

            apply_solution(solve_results, x);

            energy_term_list_.apply_post_processing();

            if (linear_solve) // if linear -> done
            {
                function_value_ = energy_term_list_.evaluate();
                break;
            }
            else // if non-linear -> check termination criteria
            {
                function_value_ = error_total_after_third_loop = energy_term_list_.evaluate();
                conv_val_rel2 = std::abs(error_total_after_third_loop - error_total_before_third_loop) / error_total_before_third_loop;

                if (conv_val_rel2 < conv_val)
                    break;

                error_total_before_third_loop = error_total_after_third_loop;
            }
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


double
Energy::
get_function_value()
{
    return function_value_;
}


//-----------------------------------------------------------------------------


double
Energy::
evaluate_fitting()
{
    return energy_term_list_.evaluate_fitting();
}


//-----------------------------------------------------------------------------


unsigned int
Energy::
compute_offsets_and_return_number_of_columns( const std::set<Solve_result>& solve_results, unsigned int nv, bool use_small_lgs, unsigned int pca_dim )
{
    unsigned int  n          = 0;
    unsigned int  offset_tmp = 0;

    Column_offsets& column_offsets = energy_term_data_.column_offsets;

    if (solve_results.count(SR_VERTICES) > 0)
    {
        column_offsets.offset_match = offset_tmp;

        if (use_small_lgs)
        {
            n += nv;
        }
        else
        {
            n += 3*nv;
        }

        offset_tmp = n;
    }

    if (solve_results.count(SR_GLOBAL_ROT_TRANS) > 0)
    {
        column_offsets.offset_nrd_w_rigid = offset_tmp;

        n += 6;

        offset_tmp = n;
    }

    if (solve_results.count(SR_PCA) > 0)
    {
        column_offsets.offset_nrd_w_pca = offset_tmp;

        n += pca_dim;

        offset_tmp = n;
    }

    //TODO consider this case
//    if (weights_settings.cb_reg_arap ||
//        weights_settings.cb_reg_hybrid_arap)
//    {
//        column_offsets.offset_nrd_w_arap = offset_tmp;

//        n += 3*nv;

//        offset_tmp = n;
//    }

    return n;
}


//-----------------------------------------------------------------------------


void
Energy::
apply_solution(const std::set<Solve_result>& solve_results, const Eigen::MatrixXd &X)
{
    Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    auto   template_points = template_mesh->vertex_property<Point>("v:point");
    const  Column_offsets& column_offsets = energy_term_data_.column_offsets;

    //std::cout << "Energy::apply_solution (small SLE): [INFO] Applying solution for" << std::endl;

    unsigned int i,j;
    if ( solve_results.count(SR_VERTICES) > 0 )
    {
        //std::cout << " -VERTICES" << std::endl;
        i = 0;
        for (auto v : template_mesh->vertices())
        {
            for (j = 0; j < 3; ++j)
            {
                template_points[v][j] = X(column_offsets.offset_match + i, j);
            }
            ++i;
        }
    }

    if (solve_results.count(SR_PCA))
    {
        //TODO do we solve for pca with small sle?
        std::cerr << "Energy::apply_solution(): [WARNING] Solving for PCA with small linear system is not supported!" << std::endl;
    }
}


//-----------------------------------------------------------------------------


void
Energy::
apply_solution(const std::set<Solve_result> &solve_results, const Eigen::VectorXd &x)
{
    Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    auto   template_points = template_mesh->vertex_property<Point>("v:point");
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    //std::cout << "Energy::apply_solution (big SLE): [INFO] Applying solution for" << std::endl;

    unsigned int i,j;
    if ( solve_results.count(SR_VERTICES) > 0 )
    {
        //std::cout << " -VERTICES" << std::endl;
        // copy solution to mesh vertices
        i = 0;
        for (auto v : template_mesh->vertices())
        {
            for (j = 0; j < 3; ++j)
            {
                template_points[v][j] = x(column_offsets.offset_match + 3*i + j);
            }
            ++i;
        }
    }

    if ( solve_results.count(SR_PCA) > 0 )
    {
        //std::cout << " -PCA" << std::endl;
        PCA& pca = energy_term_data_.pca;

        for (j = 0; j < pca.dim_pca_model_; ++j)
        {
            pca.d_pca_(j) = x(column_offsets.offset_nrd_w_pca + j);
        }

        if (solve_results.count(SR_VERTICES) == 0)
        {
            Eigen::VectorXd newShape = pca.P_pca_ * pca.d_pca_ + pca.m_pca_;
            i = 0;
            for (auto v : template_mesh->vertices())
            {
                for (j = 0; j < 3; ++j)
                {
                    template_points[v][j] = newShape(3*i + j);
                }
                ++i;
            }
        }
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
