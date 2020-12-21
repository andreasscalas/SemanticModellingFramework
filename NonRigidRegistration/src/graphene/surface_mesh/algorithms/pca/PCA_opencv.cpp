//== INCLUDES =================================================================


#include "PCA_opencv.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


template<class T>
void save_pca(const std::string filename, const T& x)
{
    cv::FileStorage f(filename.c_str(), cv::FileStorage::WRITE);

    f << "pca-model" << x;
    f.release();
}


//-----------------------------------------------------------------------------


template <class T>
T load_pca(const std::string filename)
{
    T x;
    cv::FileStorage f(filename.c_str(), cv::FileStorage::READ);

    f["pca-model"] >> x;
    f.release();
    return x;
}


//-----------------------------------------------------------------------------


PCA_opencv::
PCA_opencv()
{}


//-----------------------------------------------------------------------------


PCA_opencv::
~PCA_opencv()
{}


//-----------------------------------------------------------------------------


bool
PCA_opencv::
train(const Eigen::MatrixXd& data, const int max_components)
{
    // check input parameters
    assert (max_components >= 1);

    unsigned int dim_vectors = data.rows();
    unsigned int num_vectors = data.cols();

    // Fill opencv matrix
    cv::Mat cv_data = cv::Mat(dim_vectors, num_vectors, CV_64F);
    for(unsigned int i = 0; i < dim_vectors; ++i)
    {
        for(unsigned int j = 0; j < num_vectors; ++j)
        {
            cv_data.at<double>(i, j) = data(i, j);
        }
    }

    pca_ocv_ = cv::PCA(cv_data,   // pass the data
                       cv::Mat(), // we do not have a pre-computed mean vector,
                                  // so let the PCA engine to compute it
                       CV_PCA_DATA_AS_COL, // indicate that the vectors
                                           // are stored as matrix columns
                       max_components // specify, how many principal components to retain
                      );

    dim_pca_model_ = (unsigned int) pca_ocv_.eigenvectors.rows;
    d_pca_.resize(dim_pca_model_);
    d_pca_.setZero();
    P_pca_ = get_bases();
    m_pca_ = get_mean();
    eigenvalues_pca_     = get_eigenvalues();
    inv_eigenvalues_pca_ = get_inv_eigenvalues();

    return true;
}


//-----------------------------------------------------------------------------


bool
PCA_opencv::
save_pca_model(const std::string filename)
{
    // write PCA model
    std::cout << "[INFO] Writing PCA_opencv object..." << std::endl;
    save_pca<PCA_opencv>(filename.c_str(), *this);

    return true;
}


//-----------------------------------------------------------------------------


bool
PCA_opencv::
load_pca_model(const std::string filename)
{
    //std::cout << "[INFO] Loading PCA_opencv object..." << std::endl;

    // load PCA model
    *this = load_pca<PCA_opencv>(filename.c_str());

    dim_pca_model_ = pca_ocv_.eigenvectors.rows;

    // get/build PCA-matrix
    P_pca_ = get_bases();

    // get/build m vector
    m_pca_ = get_mean();

    d_pca_.resize(dim_pca_model_);
    d_pca_.setZero();

    // get/build eigenvalues
    eigenvalues_pca_ = get_eigenvalues();
    inv_eigenvalues_pca_ = get_inv_eigenvalues();

    // sum_inverse_eigenvalues_ = 0.0;
    // for (unsigned int i = 0; i < dim_pca_model_; ++i)
    // {
    //     sum_inverse_eigenvalues_ += inv_eigenvalues_pca_(i);
    //     std::cerr << "inv_eigenvalues_pca_(i): " << inv_eigenvalues_pca_(i) << std::endl;
    // }
    // std::cerr << "sum_inverse_eigenvalues_: " << sum_inverse_eigenvalues_ << std::endl;

    return true;
}


//-----------------------------------------------------------------------------


void
PCA_opencv::
project(const Eigen::VectorXd vec, Eigen::VectorXd& result) const
{
    unsigned int dim_vec = vec.size();

    cv::Mat cv_vec(dim_vec, 1, CV_64F);
    for (unsigned int i = 0; i < dim_vec; ++i)
        cv_vec.at<double>(i, 0) = vec(i);
    cv::Mat cv_coeffs;

    // project
    pca_ocv_.project(cv_vec, cv_coeffs);

    const unsigned int dim_result = cv_coeffs.rows;

    result.resize(dim_result);
    for (unsigned int i = 0; i < dim_result; ++i)
        result(i) = cv_coeffs.at<double>(i, 0); // TODO correct?!
}


//-----------------------------------------------------------------------------


void
PCA_opencv::
back_project(const Eigen::VectorXd vec, Eigen::VectorXd& result) const
{
    unsigned int dim_vec    = vec.size();

    cv::Mat cv_vec(dim_vec, 1, CV_64F);
    for (unsigned int i = 0; i < dim_vec; ++i)
        cv_vec.at<double>(i, 0) = vec(i);
    cv::Mat cv_back;

    // project
    pca_ocv_.backProject(cv_vec, cv_back);

    const unsigned int dim_result = cv_back.rows;

    result.resize(dim_result);
    for (unsigned int i = 0; i < dim_result; ++i)
        result(i) = cv_back.at<double>(i, 0);
}


//-----------------------------------------------------------------------------


void
PCA_opencv::
write(cv::FileStorage &fs) const
{
    assert(fs.isOpened());
    fs 
        << "{"
        << "evec" << pca_ocv_.eigenvectors
        << "eval" << pca_ocv_.eigenvalues
        << "mean" << pca_ocv_.mean
        << "}";
}


//-----------------------------------------------------------------------------


void
PCA_opencv::
read(const cv::FileNode& node)
{
    assert(node.type() == cv::FileNode::MAP);
    node["evec"] >> pca_ocv_.eigenvectors;
    node["eval"] >> pca_ocv_.eigenvalues;
    node["mean"] >> pca_ocv_.mean;
}


//-----------------------------------------------------------------------------


Eigen::VectorXd
PCA_opencv::
get_mean()
{
    int m = pca_ocv_.mean.rows;

    Eigen::VectorXd result(m);
    for (unsigned int i = 0; i < m; ++i)
    {
        result(i) = pca_ocv_.mean.at<double>(i, 0);
    }

    return result;
}


//-----------------------------------------------------------------------------


Eigen::MatrixXd
PCA_opencv::
get_bases(bool divide_by_eigenvalue)
{
    int m = pca_ocv_.eigenvectors.cols;
    int n = pca_ocv_.eigenvectors.rows;

    Eigen::MatrixXd result(m, n);
    for (unsigned int i = 0; i < m; ++i)
    {
        for (unsigned int j = 0; j < n; ++j)
        {
            result(i, j) = pca_ocv_.eigenvectors.at<double>(j, i);
        }
    }

    if (divide_by_eigenvalue)
    {
        for (unsigned int j = 0; j < n; ++j)
        {
            double current_eigenvalue = pca_ocv_.eigenvalues.at<double>(j);

            for (unsigned int i = 0; i < m; ++i)
            {
                result(i, j) /= current_eigenvalue;
            }
        }
    }

    return result;
}


//-----------------------------------------------------------------------------


Eigen::VectorXd
PCA_opencv::
get_eigenvalues()
{
    const unsigned int n = pca_ocv_.eigenvectors.rows;

    Eigen::VectorXd result(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        result(i) = pca_ocv_.eigenvalues.at<double>(i);
    }

    return result;
}


//-----------------------------------------------------------------------------


Eigen::VectorXd
PCA_opencv::
get_inv_eigenvalues()
{
    const unsigned int n = pca_ocv_.eigenvectors.rows;

    Eigen::VectorXd result(n);
    result.setZero();
    for (unsigned int i = 0; i < n; ++i)
    {
        result(i) = 1.0 / pca_ocv_.eigenvalues.at<double>(i);
    }

    return result;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
