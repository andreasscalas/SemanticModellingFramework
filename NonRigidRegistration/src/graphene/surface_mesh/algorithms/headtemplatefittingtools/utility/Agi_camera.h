//=============================================================================


#ifndef GRAPHENE_AGI_CAMERA_H
#define GRAPHENE_AGI_CAMERA_H


//== INCLUDES =================================================================


#include <string>
#include <vector>
#include <map>

#include <graphene/types.h>

#include <graphene/geometry/Matrix3x3.h>
#include <graphene/geometry/Matrix4x4.h>

#include <Eigen/Dense>
#include <tinyxml/tinyxml.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== CLASS DEFINITION =========================================================


class
Agi_camera
{
public:
    struct Sensor
    {
        std::string id_;
        int         width_;
        int         height_;
        double      f_;
        double      fx_; // for old camera xml file format
        double      fy_; // for old camera xml file format
        double      cx_;
        double      cy_;
        double      b1_;
        double      b2_;
        double      k1_;
        double      k2_;
        double      k3_;
        double      p1_;
        double      p2_;
        double      skew_; // for old camera xml file format

        Sensor()
        {
            width_ = height_ = 0;
            f_=fx_=fy_=cx_=cy_=b1_=b2_=k1_=k2_=k3_=p1_=p2_=skew_=0.0;
        }
    };

public:

    Agi_camera();

    ~Agi_camera();

    bool read_camera(TiXmlElement* camera_element, const std::map<std::string, Sensor>& sensors, int file_version);

    bool init(Eigen::Matrix3d& mat_R, Eigen::Vector3d& vec_t, double fx, double fy, double cx, double cy, double skew);

    Vec2d world_to_pixel( const Point& point_world ) const;

    Vec2f world_to_pixel_swenninger( const Point& point_world ) const;

    // get ray center and ray direction and shoot ray from camera center through pixel center
    void shoot_ray( double pixel_coord_x, double pixel_coord_y, Eigen::Vector3d& ray_center, Eigen::Vector3d& ray_direction ) const;

    Point get_center() const;

    bool is_initialized() const { return initialized_; }

    graphene::Mat4d get_mat_R()
    {
        Mat4d my_mat_R = Mat4d::identity();
        my_mat_R(0,0) = mat_R_(0,0);
        my_mat_R(1,0) = mat_R_(1,0);
        my_mat_R(2,0) = mat_R_(2,0);
        my_mat_R(0,1) = mat_R_(0,1);
        my_mat_R(1,1) = mat_R_(1,1);
        my_mat_R(2,1) = mat_R_(2,1);
        my_mat_R(0,2) = mat_R_(0,2);
        my_mat_R(1,2) = mat_R_(1,2);
        my_mat_R(2,2) = mat_R_(2,2);

        return my_mat_R;
    }

    graphene::Mat4d get_mat_Rt() const
    {
        Mat4d my_mat_Rt = Mat4d::identity();
        my_mat_Rt(0,0) = mat_Rt_(0,0);
        my_mat_Rt(1,0) = mat_Rt_(1,0);
        my_mat_Rt(2,0) = mat_Rt_(2,0);
        my_mat_Rt(3,0) = 0.0;
        my_mat_Rt(0,1) = mat_Rt_(0,1);
        my_mat_Rt(1,1) = mat_Rt_(1,1);
        my_mat_Rt(2,1) = mat_Rt_(2,1);
        my_mat_Rt(3,1) = 0.0;
        my_mat_Rt(0,2) = mat_Rt_(0,2);
        my_mat_Rt(1,2) = mat_Rt_(1,2);
        my_mat_Rt(2,2) = mat_Rt_(2,2);
        my_mat_Rt(3,2) = 0.0;
        my_mat_Rt(0,3) = mat_Rt_(0,3);
        my_mat_Rt(1,3) = mat_Rt_(1,3);
        my_mat_Rt(2,3) = mat_Rt_(2,3);
        my_mat_Rt(3,3) = 1.0;

        return my_mat_Rt;
    }

    graphene::Vec4d get_vec_t()
    {
        Vec4d my_vec_t;
        my_vec_t[0] = vec_t_(0);
        my_vec_t[1] = vec_t_(1);
        my_vec_t[2] = vec_t_(2);
        my_vec_t[3] = 1.0f;

        return my_vec_t;
    }

    graphene::Mat4d get_mat_K() const
    {
        Mat4d my_mat_K = Mat4d::identity();
        my_mat_K(0,0) = mat_K_(0,0);
        my_mat_K(1,0) = mat_K_(1,0);
        my_mat_K(2,0) = mat_K_(2,0);
        my_mat_K(0,1) = mat_K_(0,1);
        my_mat_K(1,1) = mat_K_(1,1);
        my_mat_K(2,1) = mat_K_(2,1);
        my_mat_K(0,2) = mat_K_(0,2);
        my_mat_K(1,2) = mat_K_(1,2);
        my_mat_K(2,2) = mat_K_(2,2);

        return my_mat_K;
    }

    int get_width() const
    {
        return sensor_.width_;
    }

    int get_height() const
    {
        return sensor_.height_;
    }

    std::string get_label_long() const
    {
        return label_long_;
    }



private:

    Eigen::Matrix3d mat_R_;

    Eigen::MatrixXd mat_Rt_;

    Eigen::Vector3d vec_t_;

    Eigen::Matrix3d mat_K_;

    bool initialized_;

    std::string label_long_;

    Sensor sensor_;

};


class
Agi_cameras
{
public:

    // load all Agi_cameras from file
    Agi_cameras(const std::string& filename_cameras);

    ~Agi_cameras();

    std::vector<std::string> get_labels_long();

    const std::vector<Agi_camera>& get_cameras() { return cameras_; }

    Agi_camera* get_camera_by_label(const std::string& label);

    Agi_camera* get_camera_by_filename(const std::string& filename);


private:

    int file_version_;

    std::vector<Agi_camera> cameras_;

    //maps sensor id to sensor
    std::map<std::string, Agi_camera::Sensor> sensors_;

    //helper

    bool read_sensor(TiXmlElement* sensor_element);

    bool read_camera(TiXmlElement* camera_element);

};


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_AGI_CAMERA_H
//=============================================================================
