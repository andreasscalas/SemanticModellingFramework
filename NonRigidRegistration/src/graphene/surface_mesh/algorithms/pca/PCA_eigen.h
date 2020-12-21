//=============================================================================


#ifndef GRAPHENE_PCA_EIGEN_H
#define GRAPHENE_PCA_EIGEN_H


//== INCLUDES =================================================================


#include "PCA.h"

#include <Eigen/Dense>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


/// Principal Component Analysis
class PCA_eigen : public PCA
{
public: //---------------------------------------------------- public functions

    /// default constructor
    PCA_eigen();

    virtual ~PCA_eigen();

    /// perform PCA
    virtual bool train(const Eigen::MatrixXd& data, const int max_components) override;

    /// save PCA model
    virtual bool save_pca_model(const std::string dirname) override;
    /// load PCA model
    virtual bool load_pca_model(const std::string dirname) override;
};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_PCA_EIGEN_H
//=============================================================================
