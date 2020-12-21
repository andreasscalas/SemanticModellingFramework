//=============================================================================


#ifndef GRAPHENE_PCA_H
#define GRAPHENE_PCA_H


//== INCLUDES =================================================================


#include <Eigen/Dense>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


/// Principal Component Analysis
class PCA
{
public: //---------------------------------------------------- public functions

    /// default constructor
    PCA();

    virtual ~PCA();

    virtual bool train(const Eigen::MatrixXd& data, const int max_components) = 0;

    virtual void clear() final;
    virtual bool save_pca_model(const std::string filename);
    virtual bool load_pca_model(const std::string filename);

    virtual void dump_eigenvalues() const final;
    virtual void dump_dimensions() const final;


public: //--------------------------------------------------------- public data

    unsigned int dim_pca_model_ = 0;

    // P matrix
    Eigen::MatrixXd P_pca_; // note that eigenvectors have length one and are ordered column-wise
    // mean model vector
    Eigen::VectorXd m_pca_;
    // PCA coefficients
    Eigen::VectorXd d_pca_;
    // PCA eigenvalues
    Eigen::VectorXd eigenvalues_pca_;
    Eigen::VectorXd inv_eigenvalues_pca_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_PCA_H
//=============================================================================
