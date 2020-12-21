//== INCLUDES ===================================================================


#include "Channel_mean_shift.h"


//== IMPLEMENTATION ============================================================


Channel_mean_shift::
Channel_mean_shift(const cv::Mat& src,
                   cv::Mat& tar,
                   const cv::Mat& src_mask,
                   const cv::Mat& tar_mask,
                   const cv::Mat& ref_mask,
                   const bool invert_direction)
{
    // compute source mean
    double num_elements_src = 0.0;
    double mean_src = 0.0;
    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            if (src_mask.at<uchar>(i, j) == 255)
            {
                mean_src += (unsigned int) src.at<uchar>(i, j);
                num_elements_src++;
            }
        }
    }
    mean_src /= num_elements_src;
    // std::cerr << "[DEBUG] mean_src: " << mean_src << std::endl;

    // compute target mean
    double num_elements_tar = 0.0;
    double mean_tar = 0.0;
    for (int i = 0; i < tar.rows; i++)
    {
        for (int j = 0; j < tar.cols; j++)
        {
            if (ref_mask.at<uchar>(i, j) == 255)
            {
                mean_tar += (unsigned int) tar.at<uchar>(i, j);
                num_elements_tar++;
            }
        }
    }
    mean_tar /= num_elements_tar;
    // std::cerr << "[DEBUG] mean_tar: " << mean_tar << std::endl;

    // mapping
    double mapping_delta;
    if (invert_direction)
    {
        mapping_delta = mean_tar - mean_src;
    }
    else
    {
        mapping_delta = mean_src - mean_tar;
    }

    // modify tar image
    for (int i = 0; i < tar.rows; i++)
    {
        for (int j = 0; j < tar.cols; j++)
        {
            const unsigned int old_val = (unsigned int) tar.at<uchar>(i, j);

            const double new_val = (unsigned int) (old_val + mapping_delta);

            if (tar_mask.at<uchar>(i, j) == 255)
            {
                tar.at<uchar>(i, j) = cv::saturate_cast<uchar>(new_val);
            }
        }
    }
}


//------------------------------------------------------------------------------


Channel_mean_shift::
Channel_mean_shift(const unsigned int mean_channel,
                   cv::Mat& tar,
                   const cv::Mat& tar_mask,
                   const cv::Mat& ref_mask)
{
    // compute target mean
    double num_elements_tar = 0.0;
    double mean_tar = 0.0;
    for (int i = 0; i < tar.rows; i++)
    {
        for (int j = 0; j < tar.cols; j++)
        {
            if (ref_mask.at<uchar>(i, j) == 255)
            {
                mean_tar += (unsigned int) tar.at<uchar>(i, j);
                num_elements_tar++;
            }
        }
    }
    mean_tar /= num_elements_tar;
    // std::cerr << "[DEBUG] mean_tar: " << mean_tar << std::endl;

    // mapping
    const double mapping_delta = mean_channel - mean_tar;

    // modify tar image
    for (int i = 0; i < tar.rows; i++)
    {
        for (int j = 0; j < tar.cols; j++)
        {
            const unsigned int old_val = (unsigned int) tar.at<uchar>(i, j);

            const double new_val = (unsigned int) (old_val + mapping_delta);

            if (tar_mask.at<uchar>(i, j) == 255)
            {
                tar.at<uchar>(i, j) = cv::saturate_cast<uchar>(new_val);
            }
        }
    }
}


//==============================================================================
