//== INCLUDES =================================================================


#include "Camera_ourphotoscanner.h"

#include <fstream>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== IMPLEMENTATION ===========================================================


Camera_ourphotoscanner::
Camera_ourphotoscanner(Eigen::MatrixXd mat_P)
{
    init( mat_P );
}


//-----------------------------------------------------------------------------


Camera_ourphotoscanner::
Camera_ourphotoscanner(const std::string& filename_camera)
{
    Eigen::MatrixXd my_mat_P(3, 4);
    std::ifstream ifs(filename_camera);
    if( ifs.is_open() != true )
    {
        std::cerr << "KONNTE DATEI NICHT OEFFNEN! " << filename_camera << std::endl;
        exit(1);
    }
    double c1, c2, c3, c4;
    unsigned int row = 0;
    while (true)
    {
        ifs >> c1 >> c2 >> c3 >> c4;
        if (ifs.eof()) break;
        my_mat_P(row, 0) = c1;
        my_mat_P(row, 1) = c2;
        my_mat_P(row, 2) = c3;
        my_mat_P(row, 3) = c4;
        ++row;
    }
    if ( row != 3 )
    {
        std::cerr << "row != 3   -> " << row << std::endl;
        exit(1);
    }

    init( my_mat_P );
}


//-----------------------------------------------------------------------------


Camera_ourphotoscanner::
~Camera_ourphotoscanner()
{}


//-----------------------------------------------------------------------------


Vec2d
Camera_ourphotoscanner::
world_to_pixel( const Point& point_world )
{
    Eigen::Vector4d point_world_eigen( point_world[0], point_world[1], point_world[2], 1.0 );

    // project
    Eigen::Vector3d pixel_image_plane = mat_P_ * point_world_eigen;
    // std::cerr << "pixel_image_plane[0]: " << pixel_image_plane[0] << std::endl;
    // std::cerr << "pixel_image_plane[1]: " << pixel_image_plane[1] << std::endl;
    // std::cerr << "pixel_image_plane[2]: " << pixel_image_plane[2] << std::endl;
    pixel_image_plane[0] /= pixel_image_plane[2];
    pixel_image_plane[1] /= pixel_image_plane[2];
    pixel_image_plane[2] /= pixel_image_plane[2];
    // std::cerr << "pixel_image_plane[0]: " << pixel_image_plane[0] << std::endl;
    // std::cerr << "pixel_image_plane[1]: " << pixel_image_plane[1] << std::endl;
    // std::cerr << "pixel_image_plane[2]: " << pixel_image_plane[2] << std::endl;

    Vec2d pixel_result;
    pixel_result[0] = pixel_image_plane[0];
    pixel_result[1] = pixel_image_plane[1];

    return pixel_result;
}


//-----------------------------------------------------------------------------


void
Camera_ourphotoscanner::
shoot_ray( double pixel_coord_x, double pixel_coord_y, Eigen::Vector3d& ray_center, Eigen::Vector3d& ray_direction )
{
    ray_center    = - mat_M_.inverse() * vec_p4_;

    Eigen::Vector3d myPixel_3D;
    myPixel_3D(0) = pixel_coord_x;
    myPixel_3D(1) = pixel_coord_y;
    myPixel_3D(2) = 1;
    ray_direction = mat_M_.inverse() * myPixel_3D;
}


//-----------------------------------------------------------------------------


void
Camera_ourphotoscanner::
init(Eigen::MatrixXd& mat_P)
{
    if ( 3 != mat_P.rows() )
    {
        std::cerr << "3 != mat_P.rows()" << std::endl;
        exit(1);
    }
    if ( 4 != mat_P.cols() )
    {
        std::cerr << "4 != mat_P.cols()" << std::endl;
        exit(1);
    }

    mat_P_ = mat_P;

    mat_M_(0, 0) = mat_P_(0, 0);
    mat_M_(0, 1) = mat_P_(0, 1);
    mat_M_(0, 2) = mat_P_(0, 2);
    mat_M_(1, 0) = mat_P_(1, 0);
    mat_M_(1, 1) = mat_P_(1, 1);
    mat_M_(1, 2) = mat_P_(1, 2);
    mat_M_(2, 0) = mat_P_(2, 0);
    mat_M_(2, 1) = mat_P_(2, 1);
    mat_M_(2, 2) = mat_P_(2, 2);

    vec_p4_(0) = mat_P_(0, 3);
    vec_p4_(1) = mat_P_(1, 3);
    vec_p4_(2) = mat_P_(2, 3);
}


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
