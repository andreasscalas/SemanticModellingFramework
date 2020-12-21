//== INCLUDES ===================================================================


#include "Poisson_image_editing.h"

#include <vector>
#include <climits>

#ifdef HAVE_CHOLMOD
#include <Eigen/CholmodSupport>
#define SparseSPDSolver CholmodSupernodalLLT
#else
#define SparseSPDSolver SimplicialLDLT
#endif


//== IMPLEMENTATION ============================================================


Poisson_image_editing::
Poisson_image_editing(const cv::Mat& source_image,
                      const cv::Mat& source_mask,
                      const cv::Mat& target_image,
                      const unsigned int target_pos_x,
                      const unsigned int target_pos_y,
                      cv::Mat& result_image,
                      const cv::Mat& dirichlet_mask,
                      const std::vector<std::vector<std::vector<std::array<int, 2> > > > &mapping_neighborhood)
  : xmin_(INT_MAX), xmax_(0), ymin_(INT_MAX), ymax_(0)
{
    // sanity check
    if (!sanity_check(source_image, source_mask, dirichlet_mask, mapping_neighborhood))
    {
        std::cerr << "Sanity check failed! Aborting..." << std::endl;
        return;
    }

    // compute ROI
    for (int i = 0; i < source_mask.rows; i++)
    {
        for (int j = 0; j < source_mask.cols; j++)
        {
            if ( source_mask.at<uchar>(i, j) == 255 )
            {
                xmin_ = std::min(xmin_, j);
                xmax_ = std::max(xmax_, j);
                ymin_ = std::min(ymin_, i);
                ymax_ = std::max(ymax_, i);
            }
        }
    }
    const int roi_mid_x = xmin_ + (xmax_ - xmin_ + 1)/2;
    const int roi_mid_y = ymin_ + (ymax_ - ymin_ + 1)/2;
    const int diff_x = target_pos_x - roi_mid_x;
    const int diff_y = target_pos_y - roi_mid_y;

    cv::Mat src = source_image.clone();
    cv::Mat tar = target_image.clone();

    src.convertTo(src, CV_64FC3);
    tar.convertTo(tar, CV_64FC3);

    // split the channels for the source and target image
    std::vector<cv::Mat> src_bgr;
    cv::split(src, src_bgr);
    cv::Mat src_b = src_bgr[0];
    cv::Mat src_g = src_bgr[1];
    cv::Mat src_r = src_bgr[2];
    std::vector<cv::Mat> tar_bgr;
    cv::split(tar, tar_bgr);
    cv::Mat tar_b = tar_bgr[0];
    cv::Mat tar_g = tar_bgr[1];
    cv::Mat tar_r = tar_bgr[2];

    std::vector<cv::Mat> result_bgr;

    // segment mask to: outside, boundary, inside
    cv::Mat segmentation;
    segment_mask(source_mask, dirichlet_mask, segmentation);

    // get inner coordinated (these pixels should be changed)
    std::vector<cv::Point> inner_coordinates;
    get_inner_coordinates(segmentation, inner_coordinates);

    // get mapping from pixel coordinate to index of inner coordinates
    std::map< unsigned int, std::map<unsigned int, unsigned int> > mapping_pixelcoordinate_to_idx;
    get_mapping_pixelcoordinate_to_idx(segmentation, mapping_pixelcoordinate_to_idx);

    // assemble A
    Poisson_image_editing::SpMat A_eigen = assemble_A(segmentation, inner_coordinates, mapping_pixelcoordinate_to_idx, mapping_neighborhood);

    // blend each channel separately

    // assemble b_b
    Eigen::VectorXd b_b_eigen = assemble_b(segmentation, inner_coordinates, src_b, tar_b, diff_x, diff_y, mapping_neighborhood);

    // assemble b_g
    Eigen::VectorXd b_g_eigen =  assemble_b(segmentation, inner_coordinates, src_g, tar_g, diff_x, diff_y, mapping_neighborhood);

    // assemble b_r
    Eigen::VectorXd b_r_eigen = assemble_b(segmentation, inner_coordinates, src_r, tar_r, diff_x, diff_y, mapping_neighborhood);

    // solve for new pixel colors
    Eigen::VectorXd x_b;
    Eigen::VectorXd x_g;
    Eigen::VectorXd x_r;
    const bool A_is_symmetric = (mapping_neighborhood.size() == 0);
    if (! solve_sle(A_eigen, b_b_eigen, b_g_eigen, b_r_eigen, x_b, x_g, x_r, A_is_symmetric) )
    {
        std::cerr << "Can't solve for new pixels. Aborting..." << std::endl;
        return;
    }

    cv::Mat result_b(tar_b.size(), CV_64FC1);
    cv::Mat result_g(tar_g.size(), CV_64FC1);
    cv::Mat result_r(tar_r.size(), CV_64FC1);

    solution_to_cvMat(segmentation, inner_coordinates, x_b, x_g, x_r, result_b, result_g, result_r, tar_b, tar_g, tar_r, diff_x, diff_y);

    // merge results for each channel
    result_bgr.push_back(result_b);
    result_bgr.push_back(result_g);
    result_bgr.push_back(result_r);
    cv::Mat result;
    cv::merge(result_bgr, result);

    result.convertTo(result, CV_8UC3);
    result_image = result.clone();
}


//------------------------------------------------------------------------------


const bool
Poisson_image_editing::
sanity_check( const cv::Mat& source_image, const cv::Mat& source_mask, const cv::Mat& dirichlet_mask,
              const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood ) const
{
    if (source_image.rows != source_mask.rows ||
        source_image.cols != source_mask.cols)
    {
        std::cerr << "[ERROR] source image and source mask dimensions mismatch!" << std::endl;
        return false;
    }

    if (dirichlet_mask.data)
    {
        if (source_image.rows != dirichlet_mask.rows ||
            source_image.cols != dirichlet_mask.cols)
        {
            std::cerr << "[ERROR] source image and dirichlet mask dimensions mismatch!" << std::endl;
            return false;
        }
    }

    if (mapping_neighborhood.size() != 0)
    {
        if (source_image.rows != mapping_neighborhood.size() ||
            source_image.cols != mapping_neighborhood[0].size())
        {
            std::cerr << "[ERROR] source image and mapping_neighborhood dimensions mismatch!" << std::endl;
            return false;
        }
    }

    bool on_boundary = false;
    for (int j = 0; j < source_mask.cols; j += (source_mask.cols-1))
    {
        for (int i = 0; i < source_mask.rows; i++)
        {
            if ( source_mask.at<uchar>(i, j) == 255 )
            {
                on_boundary = true;
            }
        }
    }
    for (int i = 0; i < source_mask.rows; i += (source_mask.rows-1))
    {
        for (int j = 0; j < source_mask.cols; j++)
        {
            if ( source_mask.at<uchar>(i, j) == 255 )
            {
                on_boundary = true;
            }
        }
    }

    if (on_boundary)
    {
        std::cerr << "[ERROR] source mask is active on boundary!" << std::endl;
        return false;
    }

    return true;
}


//------------------------------------------------------------------------------


void
Poisson_image_editing::
segment_mask(const cv::Mat& source_mask, const cv::Mat& dirichlet_mask, cv::Mat& segmentation) const
{
    segmentation = cv::Mat(source_mask.size(), CV_8UC1);

    // initialize to outside
    for (int i = 0; i < segmentation.rows; i++)
    {
        for (int j = 0; j < segmentation.cols; j++)
        {
            segmentation.at<uchar>(i, j) = outside;
        }
    }

    // now set pixels to inside w.r.t. source mask
    for (int i = 0; i < source_mask.rows; i++)
    {
        for (int j = 0; j < source_mask.cols; j++)
        {
            if ( source_mask.at<uchar>(i, j) == 255 )
            {
                segmentation.at<uchar>(i, j) = inside;
            }
        }
    }

    // and eventually update outer pixels to boundary
    for (int i = 0; i < source_mask.rows; i++)
    {
        for (int j = 0; j < source_mask.cols; j++)
        {
            if (segmentation.at<uchar>(i, j) == outside)
            {
                //check whether we access pixels that are outside of the image
                if (i-1 < 0 || i+1 >= source_mask.rows || j-1 < 0 || j+1 >= source_mask.cols)
                {
                    continue;
                }
                if (source_mask.at<uchar>(i-1, j-1) == 255 ||
                    source_mask.at<uchar>(i-1, j)   == 255 || 
                    source_mask.at<uchar>(i-1, j+1) == 255 ||
                    source_mask.at<uchar>(i, j-1)   == 255 ||
                    source_mask.at<uchar>(i, j+1)   == 255 ||
                    source_mask.at<uchar>(i+1, j-1) == 255 ||
                    source_mask.at<uchar>(i+1, j)   == 255 ||
                    source_mask.at<uchar>(i+1, j+1) == 255)
                {
                    segmentation.at<uchar>(i, j) = boundary;
                }
            }
        }
    }

    // dirichlet boundary conditions
    if (dirichlet_mask.data)
    {
        for (int i = 0; i < dirichlet_mask.rows; i++)
        {
            for (int j = 0; j < dirichlet_mask.cols; j++)
            {
                if ( dirichlet_mask.at<uchar>(i, j) == 255 &&
                     segmentation.at<uchar>(i, j) != inside )
                {
                    segmentation.at<uchar>(i, j) = dirichlet;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < segmentation.rows; i++)
        {
            for (int j = 0; j < segmentation.cols; j++)
            {
                if (segmentation.at<uchar>(i, j) == boundary)
                {
                    segmentation.at<uchar>(i, j) = dirichlet;
                }
            }
        }
    }
}


//------------------------------------------------------------------------------


void
Poisson_image_editing::
get_inner_coordinates(const cv::Mat& segmentation, std::vector<cv::Point>& result) const
{
    result.clear();

    for (int i = 0; i < segmentation.rows; i++)
    {
        for (int j = 0; j < segmentation.cols; j++)
        {
            if ( segmentation.at<uchar>(i, j) == inside )
            {
                result.push_back(cv::Point(j, i));
            }
        }
    }
}


//------------------------------------------------------------------------------


void
Poisson_image_editing::
get_mapping_pixelcoordinate_to_idx(const cv::Mat& segmentation, std::map< unsigned int, std::map<unsigned int, unsigned int> >& mapping) const
{
    unsigned int idx = 0;
    for (int i = 0; i < segmentation.rows; i++)
    {
        for (int j = 0; j < segmentation.cols; j++)
        {
            if ( segmentation.at<uchar>(i, j) == inside )
            {
                mapping[i][j] = idx;
                idx++;
            }
        }
    }
}


//------------------------------------------------------------------------------


Poisson_image_editing::SpMat
Poisson_image_editing::
assemble_A(const cv::Mat& segmentation, const std::vector<cv::Point>& inner_coordinates, const std::map< unsigned int, std::map<unsigned int, unsigned int> >& mapping_pixelcoordinate_to_idx, const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood) const
{
    const unsigned int dofs = inner_coordinates.size();

    Poisson_image_editing::SpMat A(dofs, dofs);
    std::vector<Tripl> coeffs;
    coeffs.reserve( 5*inner_coordinates.size() );

#if _OPENMP >= 201310
#pragma omp declare reduction (merge : std::vector<Tripl> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge: coeffs)
#endif
    for (unsigned int i = 0; i < inner_coordinates.size(); ++i)
    {
        const unsigned int x = inner_coordinates[i].x;
        const unsigned int y = inner_coordinates[i].y;

        unsigned int lx = x-1;
        unsigned int ly = y;
        unsigned int tx = x;
        unsigned int ty = y-1;
        unsigned int rx = x+1;
        unsigned int ry = y;
        unsigned int bx = x;
        unsigned int by = y+1;

        // bool jump_l = false;
        // bool jump_t = false;
        // bool jump_r = false;
        // bool jump_b = false;
        if (mapping_neighborhood.size() != 0)
        {
            // if (mapping_neighborhood[y][x][0][1] != lx) jump_l = true;
            lx = mapping_neighborhood[y][x][0][1];
            // if (mapping_neighborhood[y][x][0][0] != ly) jump_l = true;
            ly = mapping_neighborhood[y][x][0][0];
            // if (mapping_neighborhood[y][x][1][1] != tx) jump_t = true;
            tx = mapping_neighborhood[y][x][1][1];
            // if (mapping_neighborhood[y][x][1][0] != ty) jump_t = true;
            ty = mapping_neighborhood[y][x][1][0];
            // if (mapping_neighborhood[y][x][2][1] != rx) jump_r = true;
            rx = mapping_neighborhood[y][x][2][1];
            // if (mapping_neighborhood[y][x][2][0] != ry) jump_r = true;
            ry = mapping_neighborhood[y][x][2][0];
            // if (mapping_neighborhood[y][x][3][1] != bx) jump_b = true;
            bx = mapping_neighborhood[y][x][3][1];
            // if (mapping_neighborhood[y][x][3][0] != by) jump_b = true;
            by = mapping_neighborhood[y][x][3][0];
        }

        if (segmentation.at<uchar>(ty, tx) == inside && segmentation.at<uchar>(y-1, x) != dirichlet) // T
        {
            const unsigned int idx = mapping_pixelcoordinate_to_idx.at(ty).at(tx);
            coeffs.push_back( Tripl(i, idx, -1.0) );
        }

        if (segmentation.at<uchar>(by, bx) == inside && segmentation.at<uchar>(y+1, x) != dirichlet) // B
        {
            const unsigned int idx = mapping_pixelcoordinate_to_idx.at(by).at(bx);
            coeffs.push_back( Tripl(i, idx, -1.0) );
        }

        if (segmentation.at<uchar>(ly, lx) == inside && segmentation.at<uchar>(y, x-1) != dirichlet) // L
        {
            const unsigned int idx = mapping_pixelcoordinate_to_idx.at(ly).at(lx);
            coeffs.push_back( Tripl(i, idx, -1.0) );
        }

        if (segmentation.at<uchar>(ry, rx) == inside && segmentation.at<uchar>(y, x+1) != dirichlet) // R
        {
            const unsigned int idx = mapping_pixelcoordinate_to_idx.at(ry).at(rx);
            coeffs.push_back( Tripl(i, idx, -1.0) );
        }

        coeffs.push_back( Tripl(i, i, 4.0) );
    }

    A.setFromTriplets(coeffs.begin(), coeffs.end());

    return A;
}


//------------------------------------------------------------------------------


Eigen::VectorXd
Poisson_image_editing::
assemble_b(const cv::Mat& segmentation, const std::vector<cv::Point>& inner_coordinates, const cv::Mat& src_channel, const cv::Mat& tar_channel, const int diff_x, const int diff_y, const std::vector< std::vector< std::vector< std::array<int,2 > > > >& mapping_neighborhood) const
{
    const unsigned int dofs = inner_coordinates.size();

    Eigen::VectorXd b(dofs);

    for (unsigned int i = 0; i < inner_coordinates.size(); ++i)
    {
        const unsigned int x = inner_coordinates[i].x;
        const unsigned int y = inner_coordinates[i].y;

        unsigned int lx = x-1;
        unsigned int ly = y;
        unsigned int tx = x;
        unsigned int ty = y-1;
        unsigned int rx = x+1;
        unsigned int ry = y;
        unsigned int bx = x;
        unsigned int by = y+1;

        // bool jump_l = false;
        // bool jump_t = false;
        // bool jump_r = false;
        // bool jump_b = false;
        // if (mapping_neighborhood.size() != 0)
        // {
        //     if (mapping_neighborhood[y][x][0][1] != lx) jump_l = true;
        //     lx = mapping_neighborhood[y][x][0][1];
        //     if (mapping_neighborhood[y][x][0][0] != ly) jump_l = true;
        //     ly = mapping_neighborhood[y][x][0][0];
        //     if (mapping_neighborhood[y][x][1][1] != tx) jump_t = true;
        //     tx = mapping_neighborhood[y][x][1][1];
        //     if (mapping_neighborhood[y][x][1][0] != ty) jump_t = true;
        //     ty = mapping_neighborhood[y][x][1][0];
        //     if (mapping_neighborhood[y][x][2][1] != rx) jump_r = true;
        //     rx = mapping_neighborhood[y][x][2][1];
        //     if (mapping_neighborhood[y][x][2][0] != ry) jump_r = true;
        //     ry = mapping_neighborhood[y][x][2][0];
        //     if (mapping_neighborhood[y][x][3][1] != bx) jump_b = true;
        //     bx = mapping_neighborhood[y][x][3][1];
        //     if (mapping_neighborhood[y][x][3][0] != by) jump_b = true;
        //     by = mapping_neighborhood[y][x][3][0];
        // }

        // TODO EVTL. IST DAS NICHT GANZ RICHTIG MIT IMMER 4 UND IMMER ALLE DIESE NACHBARN
        b(i) = 4.0*src_channel.at<double>( y,  x) -
                   src_channel.at<double>(ty, tx) - // T
                   src_channel.at<double>(by, bx) - // B
                   src_channel.at<double>(ly, lx) - // L
                   src_channel.at<double>(ry, rx);  // R

        // T
        if (segmentation.at<uchar>(ty, tx) == dirichlet)
        {
            b(i) += tar_channel.at<double>(ty + diff_y, tx + diff_x);
        }
        else if (segmentation.at<uchar>(ty, tx) == boundary)
        {
            b(i) += src_channel.at<double>(ty + diff_y, tx + diff_x);
             // b(i) -= tar_channel.at<double>(y + diff_y, x + diff_x);
        }

        // B
        if (segmentation.at<uchar>(by, bx) == dirichlet)
        {
            b(i) += tar_channel.at<double>(by + diff_y, bx + diff_x);
        }
        else if (segmentation.at<uchar>(by, bx) == boundary)
        {
            b(i) += src_channel.at<double>(by + diff_y, bx + diff_x);
            // b(i) -= tar_channel.at<double>(y + diff_y, x + diff_x);
        }

        // L
        if (segmentation.at<uchar>(ly, lx) == dirichlet)
        {
            b(i) += tar_channel.at<double>(ly + diff_y, lx + diff_x);
        }
        else if (segmentation.at<uchar>(ly, lx) == boundary)
        {
            b(i) += src_channel.at<double>(ly + diff_y, lx + diff_x);
            // b(i) -= tar_channel.at<double>(y + diff_y, x + diff_x);
        }

        // R
        if (segmentation.at<uchar>(ry, rx) == dirichlet)
        {
           b(i) += tar_channel.at<double>(ry + diff_y, rx + diff_x);
        }
        else if (segmentation.at<uchar>(ry, rx) == boundary)
        {
           b(i) += src_channel.at<double>(ry + diff_y, rx + diff_x);
           // b(i) -= tar_channel.at<double>(y + diff_y, x + diff_x);
        }
    }

    return b;
}


//------------------------------------------------------------------------------


void
Poisson_image_editing::
solution_to_cvMat(const cv::Mat& segmentation,
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
                  const int diff_x, const int diff_y) const
{
    result_channel_b = target_channel_b.clone();
    result_channel_g = target_channel_g.clone();
    result_channel_r = target_channel_r.clone();

    for (unsigned int i = 0; i < inner_coordinates.size(); ++i)
    {
        const unsigned int x = inner_coordinates[i].x;
        const unsigned int y = inner_coordinates[i].y;

        result_channel_b.at<double>(y + diff_y, x + diff_x) = x_eigen_b(i);
        result_channel_g.at<double>(y + diff_y, x + diff_x) = x_eigen_g(i);
        result_channel_r.at<double>(y + diff_y, x + diff_x) = x_eigen_r(i);
    }
}


//------------------------------------------------------------------------------


const bool
Poisson_image_editing::
solve_sle(const SpMat& A,
          const Eigen::VectorXd& b_b,
          const Eigen::VectorXd& b_g,
          const Eigen::VectorXd& b_r,
          Eigen::VectorXd& x_b,
          Eigen::VectorXd& x_g,
          Eigen::VectorXd& x_r,
          const bool is_symmetric) const
{
    if (is_symmetric)
    {
        // Eigen::SimplicialLDLT< SpMat > solver;
        Eigen::SparseSPDSolver< SpMat > solver;

        solver.compute(A);
        if(solver.info() != Eigen::Success)
        {
            // decomposition failed
            std::cerr << "[ERROR] Decomposition failed" << std::endl;
            return false;
        }

        x_b = solver.solve(b_b);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for b-channel failed" << std::endl;
            return false;
        }

        x_g = solver.solve(b_g);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for g-channel failed" << std::endl;
            return false;
        }

        x_r = solver.solve(b_r);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for r-channel failed" << std::endl;
            return false;
        }
    }
    else
    {
        Eigen::SparseLU< SpMat > solver;

        solver.compute(A);
        if(solver.info() != Eigen::Success)
        {
            // decomposition failed
            std::cerr << "[ERROR] Decomposition failed" << std::endl;
            return false;
        }

        x_b = solver.solve(b_b);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for b-channel failed" << std::endl;
            return false;
        }

        x_g = solver.solve(b_g);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for g-channel failed" << std::endl;
            return false;
        }

        x_r = solver.solve(b_r);
        if(solver.info() != Eigen::Success)
        {
            // solving failed
            std::cerr << "[ERROR] Solving for r-channel failed" << std::endl;
            return false;
        }
    }

    return true;
}


//==============================================================================
