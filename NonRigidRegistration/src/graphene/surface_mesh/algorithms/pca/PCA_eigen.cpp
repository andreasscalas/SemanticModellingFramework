//== INCLUDES =================================================================


#include "PCA_eigen.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>

#include <iostream>

#include <Eigen/SVD>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


PCA_eigen::
PCA_eigen()
{}


//-----------------------------------------------------------------------------


PCA_eigen::
~PCA_eigen()
{}


//-----------------------------------------------------------------------------


bool
PCA_eigen::
train(const Eigen::MatrixXd& data, const int max_components)
{
    // see <http://www.vision.jhu.edu/teaching/vision08/Handouts/case_study_pca1.pdf>

    // check input parameters
    assert (max_components >= 1);

    const unsigned int dim_vectors = data.rows();
    const unsigned int num_vectors = data.cols();

    // the vectors are stored as matrix columns
    Eigen::MatrixXd A = data;

    m_pca_ = Eigen::VectorXd(data.rows());
    m_pca_ = A.rowwise().sum();
    m_pca_ /= num_vectors;

    A.colwise() -= m_pca_;

    Eigen::MatrixXd CV = A.transpose() * A / (num_vectors - 1);
    std::cerr << "CV.rows(): " << CV.rows() << std::endl;
    std::cerr << "CV.cols(): " << CV.cols() << std::endl;

    Eigen::BDCSVD<Eigen::MatrixXd> svd(CV, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd V = svd.matrixV();
    std::cerr << "V.rows(): " << V.rows() << std::endl;
    std::cerr << "V.cols(): " << V.cols() << std::endl;

    Eigen::VectorXd sv = svd.singularValues();

    Eigen::MatrixXd U = A * V;
    U.colwise().normalize();
    std::cerr << "U.rows(): " << U.rows() << std::endl;
    std::cerr << "U.cols(): " << U.cols() << std::endl;

    // how many principal components to retain
    P_pca_ = U.leftCols(max_components);

    dim_pca_model_ = (unsigned int) P_pca_.cols();
    d_pca_.resize(dim_pca_model_);
    d_pca_.setZero();
    eigenvalues_pca_ = sv.topRows(dim_pca_model_);
    inv_eigenvalues_pca_.resize(dim_pca_model_);
    for (unsigned int i = 0; i < dim_pca_model_; ++i)
    {
        inv_eigenvalues_pca_(i) = 1.0 / eigenvalues_pca_(i);
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
PCA_eigen::
save_pca_model(const std::string dirname)
{
    if (!write_eigen_matrix_as_bin(P_pca_,
                                   dirname + "pca_model_matrix.matrix",
                                   P_pca_.rows(),
                                   P_pca_.cols()))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::save_pca_model(...)' - Can't save PCA matrix" << std::endl;
        return false;
    }
    if (!write_eigen_vector_as_bin(m_pca_,
                                   dirname + "pca_model_mean.vector",
                                   m_pca_.size()))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::save_pca_model(...)' - Can't save mean" << std::endl;
        return false;
    }
    if (!write_eigen_vector_as_bin(eigenvalues_pca_,
                                   dirname + "pca_model_eigenvalues.vector",
                                   eigenvalues_pca_.size()))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::save_pca_model(...)' - Can't save eigenvalues" << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
PCA_eigen::
load_pca_model(const std::string dirname)
{
    if (!read_eigen_matrix_from_bin(P_pca_,
                                    dirname + "pca_model_matrix.matrix"))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::load_pca_model(...)' - Can't read PCA matrix" << std::endl;
        return false;
    }
    if (!read_eigen_vector_from_bin(m_pca_,
                                    dirname + "pca_model_mean.vector"))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::load_pca_model(...)' - Can't read mean" << std::endl;
        return false;
    }
    if (!read_eigen_vector_from_bin(eigenvalues_pca_,
                                    dirname + "pca_model_eigenvalues.vector"))
    {
        std::cerr << "[ERROR] in 'PCA_eigen::load_pca_model(...)' - Can't read eigenvalues" << std::endl;
        return false;
    }

    dim_pca_model_ = P_pca_.cols();

    d_pca_.resize(dim_pca_model_);
    d_pca_.setZero();

    const unsigned int n = dim_pca_model_;
    inv_eigenvalues_pca_.resize(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        inv_eigenvalues_pca_(i) = 1.0 / eigenvalues_pca_(i);
    }

    return true;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
