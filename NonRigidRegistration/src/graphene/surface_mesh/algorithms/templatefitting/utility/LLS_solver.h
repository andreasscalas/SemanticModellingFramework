//=============================================================================
#ifndef GRAPHENE_LLS_SOLVER_H
#define GRAPHENE_LLS_SOLVER_H
//=============================================================================

//== INCLUDES =================================================================


#include <vector>

#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#ifdef HAVE_CHOLMOD
#include <Eigen/CholmodSupport>
#define SparseSPDSolver CholmodSupernodalLLT
#else
#define SparseSPDSolver SimplicialLDLT
#endif


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== CLASS DEFINITION =========================================================


class LLS_solver
{

    typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<double>      Tripl;

public:

//    struct Robust_parameter
//    {
//        unsigned int  offset_idx;
//        unsigned int  num_constr;
//        double        parameter; // huber or p (for Lp)
//    };

public: //---------------------------------------------------- public functions


    // constructor
    LLS_solver();

    bool compute(const SpMat& A);
    bool solve(const Eigen::MatrixXd& B, Eigen::MatrixXd& X);
    bool solve(const Eigen::VectorXd& b, Eigen::VectorXd& x);

//    // iteratively reweighted least squares
//    void solve_irls_lp_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                                      std::vector< Robust_parameter >& robust_parameters ) const;
//    void solve_irls_lp_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                                      std::vector< Robust_parameter >& robust_parameters ) const;
//    void solve_irls_huber_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                                         std::vector< Robust_parameter >& robust_parameters ) const;
//    void solve_irls_huber_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                                         std::vector< Robust_parameter >& robust_parameters ) const;
//    void solve_irls_gemanmcclure_delta_variant( const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X,
//                                                std::vector< Robust_parameter >& robust_parameters ) const;
//    void solve_irls_gemanmcclure_delta_variant( const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x,
//                                                std::vector< Robust_parameter >& robust_parameters ) const;

    bool solve_irls(const SpMat& A, const Eigen::MatrixXd& B, const std::vector<surface_mesh::Energy_term*>& active_terms, Eigen::MatrixXd& X) const;
    bool solve_irls(const SpMat& A, const Eigen::VectorXd& b, const std::vector<surface_mesh::Energy_term*>& active_terms, Eigen::VectorXd& x) const;

    // old
    /* void solve_irls_lp(const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x, const unsigned int num_fitting_correspondences, const double p_lp_norm) const; */

private:

    void print_compute_error(Eigen::ComputationInfo info) const;

    // solve linear least squares system
    bool solve_lls(const SpMat& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& X) const;
    bool solve_lls(const SpMat& A, const Eigen::VectorXd& b, Eigen::VectorXd& x) const;

private:

    Eigen::SparseSPDSolver< SpMat >  solver_;
    SpMat                            A_;

};


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_LLS_SOLVER_H
//=============================================================================
