//=============================================================================


#ifndef GRAPHENE_PCA_OPENCV_H
#define GRAPHENE_PCA_OPENCV_H


//== INCLUDES =================================================================


#include "PCA.h"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>



//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


/// Principal Component Analysis
class PCA_opencv : public PCA
{
public: //---------------------------------------------------- public functions

    /// default constructor
    PCA_opencv();

    virtual ~PCA_opencv();

    /// perform PCA (opencv)
    virtual bool train(const Eigen::MatrixXd& data, const int max_components) override;

    virtual bool save_pca_model(const std::string filename) override;

    virtual bool load_pca_model(const std::string filename) override;

    /// projects vector from the original space to the principal components subspace
    void project(const Eigen::VectorXd vec, Eigen::VectorXd& result) const;

    /// reconstructs the original vector from the projection
    void back_project(const Eigen::VectorXd vec, Eigen::VectorXd& result) const;

    // file storage object to write to
    void write(cv::FileStorage &fs) const;

    // file storage node to read from
    void read(const cv::FileNode& node);

private:

    // get mean
    Eigen::VectorXd get_mean();

    // get base, note that eigenvectors have length one
    Eigen::MatrixXd get_bases(bool divide_by_eigenvalue = false);

    // get eigenvalues
    Eigen::VectorXd get_eigenvalues();
    Eigen::VectorXd get_inv_eigenvalues();


private: //--------------------------------------------------------- private data

    cv::PCA pca_ocv_;

};


//==============================================================================                                                                                    


static void
write(cv::FileStorage& fs, const std::string&, const PCA_opencv& x)
{
   x.write(fs);
}


//-----------------------------------------------------------------------------


static void
read(const cv::FileNode& node, PCA_opencv& x, const PCA_opencv& d)
{
    if(node.empty())
        x = d;
    else
        x.read(node);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_PCA_OPENCV_H
//=============================================================================
