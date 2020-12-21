//== INCLUDES ===================================================================


#include "LLS_solver.h"

#include <iostream>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <algorithm>

#include <Eigen/Cholesky>


#define CONVCRIT 1e-5


//== NAMESPACES ================================================================


namespace graphene {
namespace utility {


//== IMPLEMENTATION ============================================================

using namespace surface_mesh;

//-----------------------------------------------------------------------------


LLS_solver::
LLS_solver()
{
//    Eigen::initParallel();
}


//-----------------------------------------------------------------------------


bool LLS_solver::compute(const SpMat& A)
{
    A_ = A;

    SpMat AtA = A_.transpose() * A_;
    solver_.compute(AtA);
    if(solver_.info() != Eigen::Success)
    {
        // decomposition failed
        print_compute_error(solver_.info());
        std::cerr << "LLS_solver::compute(): [ERROR] Decomposition failed!" << std::endl;
        return false;
    }
    return true;
}


//-----------------------------------------------------------------------------

bool
LLS_solver::
solve(const Eigen::MatrixXd& B, Eigen::MatrixXd& X)
{
    Eigen::MatrixXd AtB = A_.transpose() * B;
    X = solver_.solve(AtB);

    if(solver_.info() != Eigen::Success)
    {
        // solving failed
        print_compute_error(solver_.info());
        std::cerr << "LLS_solver::solve(): [ERROR] Solving failed!" << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
LLS_solver::
solve(const Eigen::VectorXd& b, Eigen::VectorXd& x)
{
    Eigen::VectorXd Atb = A_.transpose() * b;
    x = solver_.solve(Atb);

    if(solver_.info() != Eigen::Success)
    {
        // solving failed
        print_compute_error(solver_.info());
        std::cerr << "LLS_solver::solve(): [ERROR] Solving failed!" << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
LLS_solver::
solve_lls(const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X) const
{
    // solve lls
    SpMat AtA = A.transpose() * A;
    Eigen::SparseSPDSolver< SpMat > solver;
    solver.compute(AtA);
    if(solver.info() != Eigen::Success)
    {
        // decomposition failed
        print_compute_error(solver.info());
        std::cerr << "LLS_solver::solve_lls: [ERROR] Decomposition failed" << std::endl;
        return false;
    }
    Eigen::MatrixXd AtB = A.transpose() * B;
    X = solver.solve(AtB);
    if(solver.info() != Eigen::Success)
    {
        // solving failed
        print_compute_error(solver.info());
        std::cerr << "LLS_solver::solve_lls: [ERROR] Solving failed" << std::endl;
        return false;
    }

    return true;
}


////-----------------------------------------------------------------------------


bool
LLS_solver::
solve_lls(const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x) const
{
    // solve lls
    SpMat AtA = A.transpose() * A;
    Eigen::SparseSPDSolver< SpMat > solver;
    solver.compute(AtA);
    if(solver.info() != Eigen::Success)
    {
        // decomposition failed
        print_compute_error(solver.info());
        std::cerr << "LLS_solver::solve_lls: [ERROR] Decomposition failed" << std::endl;
        return false;
    }
    Eigen::VectorXd Atb = A.transpose() * b;
    x = solver.solve(Atb);
    if(solver.info() != Eigen::Success)
    {
        // solving failed
        print_compute_error(solver.info());
        std::cerr << "LLS_solver::solve_lls: [ERROR] Solving failed" << std::endl;
        return false;
    }

    return true;
}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_lp_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                             std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_lp_delta_variant(Matrix B)'" << std::endl;

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::MatrixXd X_old;
//    solve_lls(A, B, X_old);

//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::MatrixXd residual = B - A*X_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double p_val = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                const double abs_res_e = 100.0*std::numeric_limits<double>::epsilon() + std::abs( residual(i) );
//                double w = std::pow( abs_res_e, p_val - 2.0 );
//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::MatrixXd AtWr    = A.transpose() * W * residual;
//        Eigen::MatrixXd X_delta = solver.solve(AtWr);

//        // update X_old
//        const double X_before_norm = X_old.norm();
//        X_old = X_old + X_delta;
//        const double X_after_norm = X_old.norm();

//        // check termination criteria
//        double my_conv_val = std::abs(X_after_norm - X_before_norm) / X_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;

//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_Lp_Delta): " << iter << std::endl;

//    X = X_old;
//}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_lp_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                             std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_lp_delta_variant(Vector b)'" << std::endl;

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::VectorXd x_old;
//    solve_lls(A, b, x_old);

//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::VectorXd residual = b - A*x_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double p_val = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                const double abs_res_e = 100.0*std::numeric_limits<double>::epsilon() + std::abs( residual(i) );
//                double w = std::pow( abs_res_e, p_val - 2.0 );
//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::VectorXd AtWr    = A.transpose() * W * residual;
//        Eigen::VectorXd x_delta = solver.solve(AtWr);

//        // update X_old
//        const double x_before_norm = x_old.norm();
//        x_old = x_old + x_delta;
//        const double x_after_norm = x_old.norm();

//        // check termination criteria
//        double my_conv_val = std::abs(x_after_norm - x_before_norm) / x_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;

//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_Lp_Delta): " << iter << std::endl;

//    x = x_old;
//}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_huber_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                                std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_huber_delta_variant(Matrix B)'" << std::endl;

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::MatrixXd X_old;
//    solve_lls(A, B, X_old);

//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::MatrixXd residual = B - A*X_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double huber_thr = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                double w  = 1;

//                Eigen::Vector3d u;
//                u(0) = residual(i, 0);
//                u(1) = residual(i, 1);
//                u(2) = residual(i, 2);
//                double norm_u = u.norm();
//                if ( norm_u > huber_thr )
//                {
//                    // std::cerr << "norm_u -> " << norm_u << std::endl;
//                    // std::cerr << "huber_thr -> " << huber_thr << std::endl;
//                    w = huber_thr / norm_u;
//                }

//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::MatrixXd AtWr    = A.transpose() * W * residual;
//        Eigen::MatrixXd X_delta = solver.solve(AtWr);

//        // update X_old
//        const double X_before_norm = X_old.norm();
//        X_old = X_old + X_delta;
//        const double X_after_norm = X_old.norm();

//        // check termination criteria
//        const double X_delta_norm  = X_delta.norm(); // TODO
////        std::cerr << "X_delta_norm -> " << X_delta_norm << std::endl;
//        double my_conv_val = std::abs(X_after_norm - X_before_norm) / X_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        // if ( X_delta_norm < 1e-5 )
//        // {
//        //     is_converged = true;
//        // }
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;

//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_Huber_Delta): " << iter << std::endl;

//    X = X_old;
//}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_huber_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                                std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_huber_delta_variant(Vector b)'" << std::endl;

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::VectorXd x_old;
//    solve_lls(A, b, x_old);

//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::VectorXd residual = b - A*x_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double huber_thr = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                double w  = 1;

//                double u;
//                u = residual(i);
//                double abs_u = std::abs(u);
//                // std::cerr << "abs_u -> " << abs_u << std::endl;
//                // std::cerr << "huber_thr -> " << huber_thr << std::endl;
//                if ( abs_u > huber_thr )
//                {
//                    w = huber_thr / abs_u;
//                }

//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::VectorXd AtWr    = A.transpose() * W * residual;
//        Eigen::VectorXd x_delta = solver.solve(AtWr);

//        // update X_old
//        const double x_before_norm = x_old.norm();
//        x_old = x_old + x_delta;
//        const double x_after_norm = x_old.norm();

//        // check termination criteria
//        double my_conv_val = std::abs(x_after_norm - x_before_norm) / x_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;

//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_Huber_Delta): " << iter << std::endl;

//    x = x_old;
//}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_gemanmcclure_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                                       std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_gemanmcclure_delta_variant(Matrix B)'" << std::endl;

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::MatrixXd X_old;
//    solve_lls(A, B, X_old);



//// BEGIN --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK

//    // initial estimate X_old
//    Eigen::MatrixXd res_HACK = B - A*X_old;

//    if ( robust_parameters.size() != 1 )
//    {
//        std::cerr << "robust_parameters.size() != 1" << std::endl;
//        exit(1);
//    }

//    // compute median
//    // see: <http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155>
//    int  ub_HACK = robust_parameters[0].offset_idx + robust_parameters[0].num_constr;
//    std::vector<double> res_vec_HACK;
//    for ( int i = robust_parameters[0].offset_idx; i < ub_HACK; ++i )
//    {
//        Eigen::Vector3d u;
//        u(0) = res_HACK(i, 0);
//        u(1) = res_HACK(i, 1);
//        u(2) = res_HACK(i, 2);

//        double norm_u = u.norm();
//        if ( norm_u != 0.0 )
//        {
//            res_vec_HACK.push_back( norm_u );
//        }
//        else
//        {
//            std::cerr << " norm u is 0  (IRLS)  !!!" << std::endl;
//            exit(1);
//        }
//    }
//    double median_HACK = 0;
//    {
//        size_t n = res_vec_HACK.size() / 2;
//        std::nth_element(res_vec_HACK.begin(), res_vec_HACK.begin()+n, res_vec_HACK.end());
//        median_HACK = res_vec_HACK[n];
//    }
//    // std::cerr << "   !!!!!!!!!!!!!!!!!!!!!!!!!!!!            median_HACK: " << median_HACK << std::endl;

//    // compute sigma_hat
//    double sigma_hat = median_HACK; // / 0.675;
////    sigma_hat = 1.0;
//    std::cerr << "   !!!!!!!!!!!!!!!!!!!!!!!!!!!!            sigma_hat: " << sigma_hat << std::endl;
//// END   --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK




//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::MatrixXd residual = B - A*X_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double scale_parameter = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                double w  = 1;

//                Eigen::Vector3d u;
//                u(0) = residual(i, 0);
//                u(1) = residual(i, 1);
//                u(2) = residual(i, 2);
//                double norm_u = u.norm();
//                // std::cerr << "norm_u: " << norm_u << std::endl;
////                w = (2.0 * scale_parameter * scale_parameter) / ( std::pow(scale_parameter*scale_parameter + norm_u*norm_u, 2) ); // VERSION 2
////                 const double aux = std::pow( norm_u/sigma_hat, 2.0 );
////                 w = 1.0 / std::pow( 1.0 + aux/(scale_parameter*scale_parameter) , 2.0 ); // VERSION 3
////                w = 1.0 / ( std::pow( 1.0 + (norm_u*norm_u / (scale_parameter*scale_parameter)) , 2) ); // VERSION 3 B


//                // const double aux = std::pow( norm_u/scale_parameter, 2.0 );
//                // w = 1.0 / std::pow( 1.0 + aux, 2.0 ); // VERSION 3 A
//                const double aux = norm_u/sigma_hat;
//                w = 1.0 / ( std::pow( 1.0 + (aux*aux/(scale_parameter*scale_parameter)), 2.0 ) ); // VERSION 3 B

//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::MatrixXd AtWr    = A.transpose() * W * residual;
//        Eigen::MatrixXd X_delta = solver.solve(AtWr);

//        // update X_old
//        const double X_before_norm = X_old.norm();
//        X_old = X_old + X_delta;
//        const double X_after_norm = X_old.norm();

//        // check termination criteria
//        const double X_delta_norm  = X_delta.norm(); // TODO
////        std::cerr << "X_delta_norm -> " << X_delta_norm << std::endl;
//        double my_conv_val = std::abs(X_after_norm - X_before_norm) / X_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        // if ( X_delta_norm < 1e-5 )
//        // {
//        //     is_converged = true;
//        // }
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;


//// BEGIN --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK

//        // Eigen::MatrixXd res_AFTER_HACK = B - A*X_old;
//        // double conv_val_HACK = ( residual - res_AFTER_HACK ).norm() / sigma_hat;
//        // std::cerr << "conv_val_HACK -> " << conv_val_HACK << std::endl;

//// END   --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK


//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_GemanMcClure_Delta): " << iter << std::endl;

//    X = X_old;
//}


////-----------------------------------------------------------------------------


//void
//LLS_solver::
//solve_irls_gemanmcclure_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                                       std::vector< Robust_parameter >& robust_parameters ) const
//{
//    std::cerr << "in: 'solve_irls_gemanmcclure_delta_variant(Vector b)'" << std::endl;
//    std::cerr << "JASCHA WAS HERE" << std::endl;
//    exit(1); // TODO

//    const unsigned int m = A.rows();
//    const unsigned int n = A.cols();

//    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
//    W.setIdentity();

//    Eigen::VectorXd x_old;
//    solve_lls(A, b, x_old);

//    const unsigned int max_iter = 50; // TODO mehrfach
//    bool is_converged = false;
//    unsigned int  iter = 0;
//    do
//    {
//        // residual
//        Eigen::VectorXd residual = b - A*x_old;

//        // update elenemts of weight-matrix
//        for ( int p = 0; p < robust_parameters.size(); ++p )
//        {
//            double scale_parameter = robust_parameters[p].parameter;
//            int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;
//            for ( int i = robust_parameters[p].offset_idx; i < upper_bound; ++i )
//            {
//                double w  = 1;

//                double u;
//                u = residual(i);
//                double abs_u = std::abs(u);
//                // w = (2.0 * scale_parameter * scale_parameter) / ( std::pow(scale_parameter*scale_parameter + abs_u*abs_u, 2) ); // VERSION 2
//                w = 1.0 / ( std::pow( 1.0 + (abs_u*abs_u / (scale_parameter*scale_parameter)) , 2) ); // VERSION 3

//                W.diagonal()[i] = w;
//            }
//        }

//        // build and solve
//        SpMat AtWA = A.transpose() * W * A;
//        Eigen::SparseSPDSolver< SpMat > solver;
//        solver.compute(AtWA);
//        if(solver.info() != Eigen::Success)
//        {
//            // decomposition failed
//            std::cerr << "decomposition failed" << std::endl;
//            exit(1);
//        }
//        Eigen::VectorXd AtWr    = A.transpose() * W * residual;
//        Eigen::VectorXd x_delta = solver.solve(AtWr);

//        // update X_old
//        const double x_before_norm = x_old.norm();
//        x_old = x_old + x_delta;
//        const double x_after_norm = x_old.norm();

//        // check termination criteria
//        double my_conv_val = std::abs(x_after_norm - x_before_norm) / x_before_norm;
//        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
//        if ( my_conv_val < CONVCRIT )
//        {
//            is_converged = true;
//        }
////        std::cerr << "termination criteria checked" << std::endl;

//        ++iter;
//    }
//    while(iter < max_iter && !is_converged);
//    std::cerr << "stopping after iteration (IRLS_GemanMcClure_Delta): " << iter << std::endl;

//    x = x_old;
//}


//-----------------------------------------------------------------------------


bool
LLS_solver::
solve_irls(const SpMat& A, const Eigen::MatrixXd& B, const std::vector<surface_mesh::Energy_term *>& active_terms, Eigen::MatrixXd& X) const
{
    const unsigned int m = A.rows();
    size_t i;
    surface_mesh::Energy_term* term;

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
    W.setIdentity();

    Eigen::MatrixXd X_old;
    if (! solve_lls(A, B, X_old))
        return false;

    const unsigned int max_iter = 50; // TODO mehrfach
    bool is_converged = false;
    unsigned int  iter = 0;
    unsigned int j;
    double w, abs_r, p;

    do
    {
        // residual
        Eigen::MatrixXd residual = B - A*X_old;

        // update elenemts of weight-matrix
        for (i=0; i < active_terms.size(); ++i)
        {
            term = active_terms[i];

            const Robustness_parameters &rp = term->get_robustness_params();

            switch (rp.solver_type)
            {

            case SOLVER_TYPE_L2:
                break;

            case SOLVER_TYPE_L1:
                p = rp.parameter;
                for ( j = term->get_row_start(); j < term->get_row_end(); ++j )
                {
                    abs_r = 100.0*std::numeric_limits<double>::epsilon() + std::abs( residual(j) );
                    w = std::pow( abs_r, p - 2.0 );
                    W.diagonal()[j] = w;
                }
                break;


            case SOLVER_TYPE_HUBER:
                p = rp.parameter;
                for ( j = term->get_row_start(); j < term->get_row_end(); ++j )
                {
                    w  = 1.0;

                    Eigen::Vector3d u;
                    u(0) = residual(j, 0);
                    u(1) = residual(j, 1);
                    u(2) = residual(j, 2);
                    const double norm_u = u.norm();
                    if ( norm_u > p )
                    {
                        // std::cerr << "norm_u -> " << norm_u << std::endl;
                        // std::cerr << "huber_thr -> " << huber_thr << std::endl;
                        w = p / norm_u;
                    }

                    W.diagonal()[j] = w;
                }
                break;


            case SOLVER_TYPE_GEMANMCCLURE:
                //TODO for jascha check gemanmcclure code and add it here
                std::cout << "LLS_solver::solve_irls(...): [WARNING] Solver type GEMANMCCLURE is not implemented! (Energy term: \"" << term->get_name() << "\")" <<std::endl;
                break;


            default:
                break;
            }
        }


        // build and solve
        SpMat AtWA = A.transpose() * W * A;
        Eigen::SparseSPDSolver< SpMat > solver;
        solver.compute(AtWA);
        if(solver.info() != Eigen::Success)
        {
            // decomposition failed
            print_compute_error(solver.info());
            std::cerr << "LLS_solver::solve_irls(...): [ERROR] Decomposition failed!" << std::endl;
            return false;
        }
        Eigen::MatrixXd AtWr    = A.transpose() * W * residual;
        Eigen::MatrixXd X_delta = solver.solve(AtWr);

        // update X_old
        const double X_before_norm = X_old.norm();
        X_old = X_old + X_delta;
        const double X_after_norm = X_old.norm();

        // check termination criteria
        //const double X_delta_norm  = X_delta.norm(); // TODO
//        std::cerr << "X_delta_norm -> " << X_delta_norm << std::endl;
        double my_conv_val = std::abs(X_after_norm - X_before_norm) / X_before_norm;
        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
        // if ( X_delta_norm < 1e-5 )
        // {
        //     is_converged = true;
        // }
        if ( my_conv_val < CONVCRIT )
        {
            is_converged = true;
        }
//        std::cerr << "termination criteria checked" << std::endl;

        ++iter;
    }
    while(iter < max_iter && !is_converged);

    X = X_old;

    return true;
}

//-----------------------------------------------------------------------------

bool LLS_solver::solve_irls(const SpMat& A, const Eigen::VectorXd& b, const std::vector<surface_mesh::Energy_term*>& active_terms, Eigen::VectorXd& x) const
{
    const unsigned int m = A.rows();
    size_t i;
    surface_mesh::Energy_term* term;

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(m);
    W.setIdentity();

    Eigen::VectorXd x_old;
    if (! solve_lls(A, b, x_old))
        return false;

    const unsigned int max_iter = 50; // TODO mehrfach
    bool is_converged = false;
    unsigned int  iter = 0;
    unsigned int j;
    double w,abs_r,p;

    do
    {
        // residual
        Eigen::VectorXd residual = b - A*x_old;

        for (i=0; i < active_terms.size(); ++i)
        {
            term = active_terms[i];
            const Robustness_parameters &rp = term->get_robustness_params();

            switch (rp.solver_type)
            {

            case SOLVER_TYPE_L2:
                break;

            case SOLVER_TYPE_L1:
                p = rp.parameter;

                //int  upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;

                for (j = term->get_row_start(); j < term->get_row_end(); ++j)
                {
                    abs_r = 100.0*std::numeric_limits<double>::epsilon() + std::abs( residual(j) );
                    w = std::pow( abs_r, p - 2.0 );
                    W.diagonal()[j] = w;
                }
                break;


            case SOLVER_TYPE_HUBER:
                p = rp.parameter;
                //upper_bound = robust_parameters[p].offset_idx + robust_parameters[p].num_constr;

                for ( j = term->get_row_start(); j < term->get_row_end(); ++j )
                {
                    w = 1.0;
                    abs_r = std::abs(residual(j));
                    // std::cerr << "abs_u -> " << abs_u << std::endl;
                    // std::cerr << "huber_thr -> " << huber_thr << std::endl;
                    if ( abs_r > p )
                    {
                        w = p / abs_r;
                    }

                    W.diagonal()[j] = w;
                }
                break;


            case SOLVER_TYPE_GEMANMCCLURE:
                std::cout << "LLS_solver::solve_irls(...): [WARNING] Solver type GEMANMCCLURE is not implemented! (Energy term: \"" << term->get_name() << "\")" <<std::endl;
                break;


            default:
                break;
            }
        }

        // build and solve
        SpMat AtWA = A.transpose() * W * A;
        Eigen::SparseSPDSolver< SpMat > solver;
        solver.compute(AtWA);
        if(solver.info() != Eigen::Success)
        {
            // decomposition failed
            print_compute_error(solver.info());
            std::cerr << "LLS_solver::solve_irls(): [ERROR] Decomposition failed" << std::endl;
            return false;
        }
        Eigen::VectorXd AtWr    = A.transpose() * W * residual;
        Eigen::VectorXd x_delta = solver.solve(AtWr);

        // update X_old
        const double x_before_norm = x_old.norm();
        x_old = x_old + x_delta;
        const double x_after_norm = x_old.norm();

        // check termination criteria
        double my_conv_val = std::abs(x_after_norm - x_before_norm) / x_before_norm;
        std::cerr << "my_conv_val -> " << my_conv_val << std::endl;
        if ( my_conv_val < CONVCRIT )
        {
            is_converged = true;
        }
//        std::cerr << "termination criteria checked" << std::endl;

        ++iter;
    }
    while(iter < max_iter && !is_converged);

    x = x_old;

    return true;
}

void
LLS_solver::
print_compute_error(Eigen::ComputationInfo info) const
{
    std::cerr << std::endl << "LLS_solver::print_compute_error: ComputationInfo" << std::endl;
    std::cerr << "    ";
    switch(info)
    {
    case Eigen::NumericalIssue:
        std::cerr << "The provided data did not satisfy the prerequisites." << std::endl;
        break;
    case Eigen::NoConvergence:
        std::cerr << "Iterative procedure did not converge." << std::endl;
        break;
    case Eigen::InvalidInput:
            std::cerr << "The inputs are invalid, or the algorithm has been improperly called." << std::endl;
        break;
    }
    std::cerr << std::endl;
}


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
