//=============================================================================
#ifndef CHANNEL_MEAN_SHIFT_H
#define CHANNEL_MEAN_SHIFT_H
//=============================================================================

//== INCLUDES =================================================================


#include "opencv2/opencv.hpp"


//== CLASS DEFINITION =========================================================


class
Channel_mean_shift
{
public:

    Channel_mean_shift(const cv::Mat& src,
                       cv::Mat& tar,
                       const cv::Mat& src_mask,
                       const cv::Mat& tar_mask,
                       const cv::Mat& ref_mask,
                       const bool invert_direction = false);

    Channel_mean_shift(const unsigned int mean_channel,
                       cv::Mat& tar,
                       const cv::Mat& tar_mask,
                       const cv::Mat& ref_mask);

};


//=============================================================================
#endif // CHANNEL_MEAN_SHIFT_H
//=============================================================================
