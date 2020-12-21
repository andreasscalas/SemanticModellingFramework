//=============================================================================


#ifndef GRAPHENE_CAMERA_OURPHOTOSCANNER_H
#define GRAPHENE_CAMERA_OURPHOTOSCANNER_H


//== INCLUDES =================================================================


#include <string>

#include <graphene/types.h>

#include <Eigen/Dense>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== CLASS DEFINITION =========================================================


class
Camera_ourphotoscanner
{
public:

    Camera_ourphotoscanner(Eigen::MatrixXd mat_P);

    // load camera of our photoscanner from file
    Camera_ourphotoscanner(const std::string& filename_camera);

    ~Camera_ourphotoscanner();

    Vec2d world_to_pixel( const Point& point_world );

    void shoot_ray( double pixel_coord_x, double pixel_coord_y, Eigen::Vector3d& ray_center, Eigen::Vector3d& ray_direction ); 

private:

    void init(Eigen::MatrixXd& mat_P);


private:

    Eigen::MatrixXd mat_P_;

    Eigen::Matrix3d mat_M_; // see hartley zissermann p.162

    Eigen::Vector3d vec_p4_;

};


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_CAMERA_OURPHOTOSCANNER_H
//=============================================================================
