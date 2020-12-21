//=============================================================================
#ifndef POISSON_IMAGE_EDITING_H
#define POISSON_IMAGE_EDITING_H
//=============================================================================

//== INCLUDES =================================================================


#include "opencv2/opencv.hpp"

#include <map>
#include <array>

#include <Eigen/Dense>
#include <Eigen/Sparse>


//== CLASS DEFINITION =========================================================


class
Poisson_image_editing
{

    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double>      Tripl;

public:

    Poisson_image_editing(const cv::Mat& source_image,
                          const cv::Mat& source_mask,
                          const cv::Mat& target_image,
                          const unsigned int target_pos_x,
                          const unsigned int target_pos_y,
                          cv::Mat& result_image,
                          const cv::Mat& dirichlet_mask = cv::Mat(),
                          const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood = std::vector< std::vector< std::vector< std::array<int, 2> > > >());

private:

    int xmin_;
    int xmax_;
    int ymin_;
    int ymax_;

    enum Segment { inside,
                   boundary,
                   dirichlet,
                   outside };

    const bool sanity_check(const cv::Mat& source_image,
                            const cv::Mat& source_mask,
                            const cv::Mat& dirichlet_mask,
                            const std::vector< std::vector< std::vector< std::array< int, 2> > > >& mapping_neighborhood) const;

    void segment_mask(const cv::Mat& source_mask, const cv::Mat& dirichlet_mask, cv::Mat& segmentation) const;

    void get_inner_coordinates(const cv::Mat& segmentation, std::vector<cv::Point>& result) const;

    void get_mapping_pixelcoordinate_to_idx(const cv::Mat& segmentation, std::map< unsigned int, std::map<unsigned int, unsigned int> >& mapping) const;

    Poisson_image_editing::SpMat assemble_A(const cv::Mat& segmentation, const std::vector<cv::Point>& inner_coordinates, const std::map< unsigned int, std::map<unsigned int, unsigned int> >& mapping_pixelcoordinate_to_idx, const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood) const;

    Eigen::VectorXd assemble_b(const cv::Mat& segmentation, const std::vector<cv::Point>& inner_coordinates, const cv::Mat& src_channel, const cv::Mat& tar_channel, const int diff_x, const int diff_y, const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood) const;

    void solution_to_cvMat(const cv::Mat& segmentation,
                           const std::vector<cv::Point>& inner_coordinates,
                           const Eigen::VectorXd& x_eigen_b,
                           const Eigen::VectorXd& x_eigen_g,
                           const Eigen::VectorXd& x_eigen_r,
                           cv::Mat& result_channel_b,
                           cv::Mat& result_channel_g,
                           cv::Mat& result_channel_r,
                           const cv::Mat& target_channel_b,
                           const cv::Mat& target_channel_g,
                           const cv::Mat& target_channel_r,
                           const int diff_x, const int diff_y) const;

    const bool solve_sle(const SpMat& A,
                         const Eigen::VectorXd& b_b,
                         const Eigen::VectorXd& b_g,
                         const Eigen::VectorXd& b_r,
                         Eigen::VectorXd& x_b,
                         Eigen::VectorXd& x_g,
                         Eigen::VectorXd& x_r,
                         const bool is_symmetric) const;

};


//=============================================================================
#endif // POISSON_IMAGE_EDITING_H
//=============================================================================
