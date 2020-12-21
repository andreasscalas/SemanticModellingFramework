//== INCLUDES =================================================================


#include "PCA.h"

#include <iostream>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


PCA::
PCA()
{}


//-----------------------------------------------------------------------------


PCA::
~PCA()
{}


//-----------------------------------------------------------------------------


void
PCA::
clear()
{
    dim_pca_model_ = 0;

    P_pca_ = Eigen::MatrixXd();
    m_pca_ = Eigen::VectorXd();
    d_pca_ = Eigen::VectorXd();
    eigenvalues_pca_ = Eigen::VectorXd();
    inv_eigenvalues_pca_ = Eigen::VectorXd();
}


//-----------------------------------------------------------------------------


bool
PCA::
save_pca_model(const std::string filename)
{
    std::cerr << "in: 'save_pca_model(...)': NOT IMPLEMENTED!" << std::endl;

    return false;
}


//-----------------------------------------------------------------------------


bool
PCA::
load_pca_model(const std::string filename)
{
    std::cerr << "in: 'load_pca_model(...)': NOT IMPLEMENTED!" << std::endl;

    return false;
}


//-----------------------------------------------------------------------------


void
PCA::
dump_eigenvalues() const
{
    double total_variance = 0.0;
    for (unsigned int i = 0; i < eigenvalues_pca_.size(); ++i)
    {
        const double ev = eigenvalues_pca_(i);
        std::cout << "eigenvalue[" << i << "] (i.e. variance): " << ev << "\t std-dev: " << sqrt(ev) << std::endl;

        total_variance += ev;
    }
    std::cerr << "total variance: " << total_variance << std::endl;

    double aux_variance = 0.0;
    for (unsigned int i = 0; i < eigenvalues_pca_.size(); ++i)
    {
        const double ev = eigenvalues_pca_(i);
        aux_variance += ev;

        double variance_distribution = aux_variance / total_variance;
        std::cerr << "variance distribution for eigenvalue[" << i << "]: " << variance_distribution << std::endl;
    }
}


//-----------------------------------------------------------------------------


void
PCA::
dump_dimensions() const
{
    std::cout << "dim_pca_model_: " << dim_pca_model_ << std::endl;

    std::cout << "P_pca_.rows(): " << P_pca_.rows() << std::endl;
    std::cout << "P_pca_.cols(): " << P_pca_.cols() << std::endl;

    std::cout << "m_pca_.size(): " << m_pca_.size() << std::endl;
    std::cout << "d_pca_.size(): " << d_pca_.size() << std::endl;

    std::cout << "eigenvalues_pca_.size(): " << eigenvalues_pca_.size() << std::endl;
    std::cout << "inv_eigenvalues_pca_.size(): " << inv_eigenvalues_pca_.size() << std::endl;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
