//=============================================================================
#ifndef COLOR_WARP_RBF_H
#define COLOR_WARP_RBF_H
//=============================================================================

//== INCLUDES =================================================================


#include <opencv2/opencv.hpp>

#include <Eigen/Dense>


//== CLASS DEFINITION =========================================================


class
Color_warp_rbf
{
public:

    struct
    Color_constraint
    {
        Eigen::Vector3d constraint_src;

        Eigen::Vector3d constraint_tar;
    };


public:

    Color_warp_rbf(const std::vector<Color_constraint>& color_constraints_lab, const bool normalize = true);

    const bool warp_colors(const cv::Mat& mask, cv::Mat& image_bgr) const;


private:

    /// RBF kernel
    inline const double kernel(const Eigen::Vector3d& c, const Eigen::Vector3d& x) const
    {
        double r = (x - c).norm();

//        return std::pow(1.0 + r, -3.7975); // normalized Shepard (eps = 3.7975) // NOT SO GOOD
//        return exp( - ( 0.0846 * r * r ) ); // Gaussian (eps = 0.0846)
//        return 0.01 + (1.0-0.01) / (1.0 + (0.0690*0.0690*r*r)); // inverse quadric


        return r;               // GOOD
//        return r*r*log(r);    // ALSO OK
//        return r * r * r;     // BAD
    }

    const Eigen::Vector3d evaluate(const Eigen::Vector3d& p) const;


private:

    /// RBF centers
    std::vector<Eigen::Vector3d>   centers_;

    /// RBF weights
    std::vector<Eigen::Vector3d>   weights_;

    bool                           normalize_;

};


//=============================================================================
#endif // COLOR_WARP_RBF_H
//=============================================================================
