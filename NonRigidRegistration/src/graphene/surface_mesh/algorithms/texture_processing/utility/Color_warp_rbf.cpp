//== INCLUDES ===================================================================


#include "Color_warp_rbf.h"

#include <vector>


//== IMPLEMENTATION ============================================================


Color_warp_rbf::
Color_warp_rbf(const std::vector<Color_constraint>& color_constraints_lab, const bool normalize)
  : normalize_(normalize)
{
    const unsigned int n = color_constraints_lab.size();

    // copy centers
    centers_.clear();
    for (unsigned int i = 0; i < n; ++i)
    {
        centers_.push_back(color_constraints_lab[i].constraint_src);
    }

    // setup system matrix
    Eigen::MatrixXd A(n, n);
    Eigen::VectorXd A_rows_sum(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        double sum_row = 0.0;
        for (unsigned int j = 0; j < n; ++j)
        {
            A(i, j) = kernel(color_constraints_lab[j].constraint_src, color_constraints_lab[i].constraint_src);
            sum_row += A(i, j);
        }
        A_rows_sum(i) = sum_row;
    }
    if (normalize_)
    {
        // ... and normalize
        for (unsigned int i = 0; i < n; ++i)
        {
            const double sum_row = A_rows_sum(i);
            for (unsigned int j = 0; j < n; ++j)
            {
                A(i, j) /= sum_row;
            }
        }
    }


    // setup rhs
    Eigen::MatrixXd diffs(n, 3);
    for (unsigned int i = 0; i < n; ++i)
    {
        const Eigen::Vector3d diff_i = color_constraints_lab[i].constraint_tar - color_constraints_lab[i].constraint_src;
        for (unsigned int j = 0; j < 3; ++j)
        {
            diffs(i, j) = diff_i(j);
        }
    }


    // solve system
    Eigen::FullPivLU<Eigen::MatrixXd> solver;
    solver.compute(A);
    Eigen::MatrixXd W = solver.solve(diffs);


    // copy weights
    weights_.clear();
    for (unsigned int i = 0; i < n; ++i)
    {
        Eigen::Vector3d w_i( W(i,0), W(i,1), W(i,2) );

        weights_.push_back( w_i );
    }
}


//------------------------------------------------------------------------------


const bool
Color_warp_rbf::
warp_colors(const cv::Mat& mask, cv::Mat& image_bgr) const
{
    cv::Mat local_image_bgr = image_bgr.clone();
    
    // convert the image to the L*a*b* color space
    cv::cvtColor(local_image_bgr, local_image_bgr, CV_BGR2Lab);

    // apply color transformation
#pragma omp parallel for
    for (int i = 0; i < local_image_bgr.rows; i++)
    {
        for (int j = 0; j < local_image_bgr.cols; j++)
        {
            if ( mask.at<uchar>(i, j) == 255 )
            {
                const float val_l = 100.0 * local_image_bgr.at<cv::Vec3b>(i, j)[0] / 255.0;
                const float val_a = local_image_bgr.at<cv::Vec3b>(i, j)[1] - 128.0;
                const float val_b = local_image_bgr.at<cv::Vec3b>(i, j)[2] - 128.0;

                const Eigen::Vector3d result = evaluate(Eigen::Vector3d( val_l, val_a, val_b ));

                cv::Vec3f new_val;
                new_val(0) = val_l + result(0);
                new_val(1) = val_a + result(1);
                new_val(2) = val_b + result(2);

                new_val(0) = 255.0 * new_val(0) / 100.0;
                new_val(1) = new_val(1) + 128.0;
                new_val(2) = new_val(2) + 128.0;

                // clamp
                if (new_val(0) < 0.0)   new_val(0) =   0.0;
                if (new_val(0) > 255.0) new_val(0) = 255.0;
                if (new_val(1) < 0.0)   new_val(1) =   0.0;
                if (new_val(1) > 255.0) new_val(1) = 255.0;
                if (new_val(2) < 0.0)   new_val(2) =   0.0;
                if (new_val(2) > 255.0) new_val(2) = 255.0;

                local_image_bgr.at<cv::Vec3b>(i, j) = cv::Vec3b(std::round(new_val(0)), std::round(new_val(1)), std::round(new_val(2))); // TODO CAST
            }
        }
    }

    // convert the image back to bgr color space
    cv::cvtColor(local_image_bgr, local_image_bgr, CV_Lab2BGR);

    image_bgr = local_image_bgr.clone();

    return true;
}


//------------------------------------------------------------------------------


const Eigen::Vector3d
Color_warp_rbf::
evaluate(const Eigen::Vector3d& p) const
{
    auto c_it(centers_.begin()), c_end(centers_.end());
    auto w_it(weights_.begin());

    Eigen::Vector3d f(0,0,0);

    double sum = 0.0;

    // accumulate RBF basis functions
    for (; c_it != c_end; ++c_it, ++w_it)
    {
        f += *w_it * kernel(*c_it, p);
        sum += kernel(*c_it, p);;
    }

    if (normalize_)
    {
        f /= sum;
    }

    return f;
}


//==============================================================================
