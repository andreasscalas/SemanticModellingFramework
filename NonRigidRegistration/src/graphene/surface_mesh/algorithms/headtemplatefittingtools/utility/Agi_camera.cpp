//== INCLUDES =================================================================


#include "Agi_camera.h"

#include "../tinyxml/tinyxml.h"

#include <sstream>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== IMPLEMENTATION ===========================================================


Agi_camera::
Agi_camera()
  : initialized_(false)
{
}




//-----------------------------------------------------------------------------


Agi_camera::
~Agi_camera()
{}



//-----------------------------------------------------------------------------

bool
Agi_camera::
read_camera(TiXmlElement *camera_element, const std::map<std::string, Sensor>& sensors, int file_version)
{
    const char* enabled = camera_element->Attribute("enabled");

    if (enabled != NULL)
    {
        const std::string s(enabled);

        if (s == "0" || s == "false")
            return false;
    }

    const char* label = camera_element->Attribute("label");
    if (label == NULL)
    {
        std::cout << "Agi_camera::read_camera: [ERROR] Camera has no label" << std::endl;
        return false;
    }
    else
    {
        label_long_ = label;
    }



    const char* sensor_id = camera_element->Attribute("sensor_id");

    std::map<std::string, Sensor>::const_iterator it;
    if (sensor_id != NULL)
    {
        it = sensors.find(sensor_id);
        if (it != sensors.end())
        {
            sensor_ = it->second;
        }
    }
    else
    {
        std::cout << "Agi_camera::read_camera(): [WARNING] Missing sensor id!" << std::endl;
    }

    Eigen::Matrix4d transform(Eigen::Matrix4d::Identity());
    TiXmlElement* el = camera_element->FirstChildElement();
    while (el != NULL)
    {
        if (el->ValueStr() == "transform")
        {
            std::stringstream ss(el->GetText());


            for (int i=0; i < 4; ++i)
            {
                for (int j=0; j < 4; ++j)
                {
                    ss >> transform(i,j);
                }
            }

        }

        el = el->NextSiblingElement();
    }

    Eigen::Matrix3d mat_R;
    Eigen::Vector3d vec_t;
    vec_t(0) = transform(0, 3);
    vec_t(1) = transform(1, 3);
    vec_t(2) = transform(2, 3);
    mat_R(0, 0) = transform(0, 0);
    mat_R(0, 1) = transform(0, 1);
    mat_R(0, 2) = transform(0, 2);
    mat_R(1, 0) = transform(1, 0);
    mat_R(1, 1) = transform(1, 1);
    mat_R(1, 2) = transform(1, 2);
    mat_R(2, 0) = transform(2, 0);
    mat_R(2, 1) = transform(2, 1);
    mat_R(2, 2) = transform(2, 2);


    if (file_version >= 140)
        return init(mat_R,
                    vec_t,
                    sensor_.f_ + sensor_.b1_,
                    sensor_.f_,
                    sensor_.cx_ + sensor_.width_/2,
                    sensor_.cy_ + sensor_.height_/2,
                    sensor_.b2_
                    );
    else
        return init(mat_R,
                    vec_t,
                    sensor_.fx_,
                    sensor_.fy_,
                    sensor_.cx_,
                    sensor_.cy_,
                    sensor_.skew_);
}


//-----------------------------------------------------------------------------


bool
Agi_camera::
init(Eigen::Matrix3d& mat_R, Eigen::Vector3d& vec_t, double fx, double fy, double cx, double cy, double skew)
{
    mat_R_ = mat_R;

    vec_t_ = vec_t;

    mat_Rt_ = Eigen::MatrixXd(3,4);
    mat_Rt_(0,0) = mat_R(0,0);
    mat_Rt_(0,1) = mat_R(0,1);
    mat_Rt_(0,2) = mat_R(0,2);
    mat_Rt_(0,3) = vec_t_(0);
    mat_Rt_(1,0) = mat_R(1,0);
    mat_Rt_(1,1) = mat_R(1,1);
    mat_Rt_(1,2) = mat_R(1,2);
    mat_Rt_(1,3) = vec_t_(1);
    mat_Rt_(2,0) = mat_R(2,0);
    mat_Rt_(2,1) = mat_R(2,1);
    mat_Rt_(2,2) = mat_R(2,2);
    mat_Rt_(2,3) = vec_t_(2);

    mat_K_(0,0) = fx;
    mat_K_(0,1) = skew;
    mat_K_(0,2) = cx;
    mat_K_(1,0) = 0.0;
    mat_K_(1,1) = fy;
    mat_K_(1,2) = cy;
    mat_K_(2,0) = 0.0;
    mat_K_(2,1) = 0.0;
    mat_K_(2,2) = 1.0;

    initialized_ = true;

    return true;
}


//-----------------------------------------------------------------------------


Vec2d
Agi_camera::
world_to_pixel( const Point& point_world ) const
{
    if (!initialized_)
    {
        std::cerr << "[ERROR] Agi_camera is not initialized." << std::endl;
        return Vec2d(0.0);
    }

    Vec2d pixel_result;

    Eigen::Vector3d point_world_eigen( point_world[0], point_world[1], point_world[2] );

    Eigen::VectorXd tmp1 = mat_R_.inverse() * ( point_world_eigen - vec_t_ );
    Eigen::VectorXd tmp2 = mat_K_ * tmp1;
    tmp2(0) /= tmp2(2);
    tmp2(1) /= tmp2(2);
    tmp2(2) /= tmp2(2);

    pixel_result[0] = tmp2(0);
    pixel_result[1] = tmp2(1);

    return pixel_result;
}


//-----------------------------------------------------------------------------

Vec2f
Agi_camera::world_to_pixel_swenninger(const Point &point_world) const
{

    Eigen::Vector3d point_world_eigen( point_world[0], point_world[1], point_world[2] );

    //
    // Transform world point to camera coordinates
    //
    Eigen::VectorXd camera_coordinates = mat_R_.inverse() * ( point_world_eigen - vec_t_ );

    //
    // Formulas here are taken from the PhotoScan User Manual for "frame"-type cameras
    //
    double X = camera_coordinates[0] / camera_coordinates[2];
    double Y = camera_coordinates[1] / camera_coordinates[2];
    double X_2 = X * X;
    double Y_2 = Y * Y;
    double XY  = X * Y;

    double r = std::sqrt(X_2 + Y_2);
    double r_2 = r * r;
    double r_4 = r_2 * r_2;
    double r_6 = r_2 * r_4;

    // radial distortion coefficient
    const Sensor& s = sensor_;
    double K_term = 1 + s.k1_ * r_2 + s.k2_ * r_4 + s.k3_ * r_6;

    // tangential distortion coefficient
    double P_term_x = s.p1_ * (r_2 + 2 * X_2) + 2 * s.p2_ * XY;
    double P_term_y = s.p2_ * (r_2 + 2 * Y_2) + 2 * s.p1_ * XY;

    double x = X * K_term + P_term_x ;
    double y = Y * K_term + P_term_y ;

    double u,v;
    u = s.cx_ + s.f_ * x + x * s.b1_ + y * s.b2_;
    v = s.cy_ + s.f_ * y;

    Vec2f pixel_result;
    pixel_result[0] = (float)u;
    pixel_result[1] = (float)v;

    return pixel_result;
}

//-----------------------------------------------------------------------------


void
Agi_camera::
shoot_ray( double pixel_coord_x, double pixel_coord_y, Eigen::Vector3d& ray_center, Eigen::Vector3d& ray_direction ) const
{
    if (!initialized_)
    {
        std::cerr << "[ERROR] Agi_camera is not initialized." << std::endl;
        return;
    }

    // center
    Eigen::VectorXd center_4D(4);
    center_4D(0) = 0.0;
    center_4D(1) = 0.0;
    center_4D(2) = 0.0;
    center_4D(3) = 1.0;
    Eigen::Vector3d center_3D = mat_Rt_ * center_4D;

    Eigen::VectorXd pixel_coord_3D(3);
    pixel_coord_3D(0) = pixel_coord_x;
    pixel_coord_3D(1) = pixel_coord_y;
    pixel_coord_3D(2) = 1.0;

    Eigen::Vector3d tmp_ray_direction = mat_K_.inverse() * pixel_coord_3D;
    tmp_ray_direction = mat_R_ * tmp_ray_direction + vec_t_;
    tmp_ray_direction = tmp_ray_direction - center_3D;
    tmp_ray_direction.normalize();

    // return results
    ray_center    = center_3D;
    ray_direction = tmp_ray_direction;
}


//-----------------------------------------------------------------------------


Point
Agi_camera::
get_center() const
{
    if (!initialized_)
    {
        std::cerr << "[ERROR] Agi_camera is not initialized." << std::endl;
        return Point(0.0f);
    }

    // center
    Eigen::VectorXd center_4D(4);
    center_4D(0) = 0.0;
    center_4D(1) = 0.0;
    center_4D(2) = 0.0;
    center_4D(3) = 1.0;
    Eigen::Vector3d center_3D = mat_Rt_ * center_4D;

    Point center;
    center[0] = center_3D(0);
    center[1] = center_3D(1);
    center[2] = center_3D(2);

    return center;
}


//=============================================================================


Agi_cameras::
Agi_cameras(const std::string& filename_cameras) :
    file_version_(0)
{

    TiXmlDocument doc;

    doc.LoadFile(filename_cameras);

    TiXmlElement* root = doc.RootElement();

    if (root == NULL)
        return;

    const char* version_str = root->Attribute("version");
    if (version_str != NULL)
    {
        std::string v(version_str);
        std::string v_nr;
        for (size_t i=0; i < v.size(); ++i)
        {
            if (v[i] != '.')
                v_nr.push_back(v[i]);
        }

        file_version_ = atoi(v_nr.c_str());
    }

    TiXmlElement* el_document_child=NULL,*el_chunk_child=NULL,*el_cameras_child=NULL;

    el_document_child = root->FirstChildElement();

    //read sensors first
    while (el_document_child != NULL) // chunks
    {

        el_chunk_child = el_document_child->FirstChildElement();
        while (el_chunk_child != NULL) // sensors/cameras
        {
            if (el_chunk_child->ValueStr() == "sensors")
            {
                TiXmlElement* el_sensors_child = el_chunk_child->FirstChildElement();
                while (el_sensors_child != NULL) // sensor
                {
                    read_sensor(el_sensors_child);

                    el_sensors_child = el_sensors_child->NextSiblingElement();
                }
            }

            el_chunk_child = el_chunk_child->NextSiblingElement();
        }


        el_document_child = el_document_child->NextSiblingElement();
    }


    //read all cameras
    el_document_child = root->FirstChildElement();
    while (el_document_child != NULL) // chunks
    {

        el_chunk_child = el_document_child->FirstChildElement();
        while (el_chunk_child != NULL) // cameras/sensors
        {
            if (el_chunk_child->ValueStr() == "cameras")
            {
                el_cameras_child = el_chunk_child->FirstChildElement();
                while (el_cameras_child != NULL) //camera
                {
                    read_camera(el_cameras_child);

                    el_cameras_child = el_cameras_child->NextSiblingElement();
                }
            }

            el_chunk_child = el_chunk_child->NextSiblingElement();
        }

        el_document_child = el_document_child->NextSiblingElement();
    }

}


//-----------------------------------------------------------------------------


Agi_cameras::
~Agi_cameras()
{}


//-----------------------------------------------------------------------------


std::vector<std::string>
Agi_cameras::
get_labels_long()
{
    std::vector<std::string> labels_long;

    for (unsigned int cam = 0; cam < cameras_.size(); ++cam)
    {
        const std::string label_long = cameras_[cam].get_label_long();

        labels_long.push_back(label_long);
    }

    return labels_long;
}


//-----------------------------------------------------------------------------

Agi_camera*
Agi_cameras::get_camera_by_label(const std::string &label)
{

    for (size_t i=0; i < cameras_.size(); ++i)
    {
        Agi_camera* c = &cameras_[i];
        if (c->get_label_long().find(label) != std::string::npos)
        {
            return c;
        }
    }

    return NULL;
}

//-----------------------------------------------------------------------------

Agi_camera*
Agi_cameras::get_camera_by_filename(const std::string &filename)
{
    for (size_t i=0; i < cameras_.size(); ++i)
    {
        Agi_camera* c = &cameras_[i];
        if (filename.find(c->get_label_long()) != std::string::npos)
        {
            return c;
        }
    }

    return NULL;
}

//-----------------------------------------------------------------------------

bool
Agi_cameras::
read_sensor(TiXmlElement *sensor_element)
{

    if (sensor_element->Attribute("id") == NULL)
    {
        std::cout << "Agi_cameras::read_sensor(): [ERROR] No ID for sensor found!" << std::endl;
        return false;
    }

    TiXmlElement* child_element = sensor_element->FirstChildElement();

    Agi_camera::Sensor &sensor = sensors_[sensor_element->Attribute("id")];

    while (child_element != NULL)
    {
        if (child_element->ValueStr() == "calibration")
        {
            TiXmlElement* el_calibration = child_element->FirstChildElement();
            while (el_calibration != NULL)
            {
                if (el_calibration->ValueStr() == "resolution")
                {
                    el_calibration->Attribute("width", &sensor.width_);
                    el_calibration->Attribute("height", &sensor.height_);
                }
                else if (el_calibration->ValueStr() == "f")
                {
                    sensor.f_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "fx")
                {
                    sensor.fx_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "fy")
                {
                    sensor.fy_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "cx")
                {
                    sensor.cx_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "cy")
                {
                    sensor.cy_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "b1")
                {
                    sensor.b1_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "b2")
                {
                    sensor.b2_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "k1")
                {
                    sensor.k1_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "k2")
                {
                    sensor.k2_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "k3")
                {
                    sensor.k3_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "p1")
                {
                    sensor.p2_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "p2")
                {
                    sensor.p2_ = atof(el_calibration->GetText());
                }
                else if (el_calibration->ValueStr() == "skew")
                {
                    sensor.skew_ = atof(el_calibration->GetText());
                }
                else
                {
                    std::cout << "Agi_cameras::read_sensor(): [WARNING] Unknown camera calibration parameter: " << el_calibration->ValueStr() << std::endl;
                }

                el_calibration = el_calibration->NextSiblingElement();
            }
        }

        child_element = child_element->NextSiblingElement();
    }

    return true;
}

//-----------------------------------------------------------------------------

bool
Agi_cameras::
read_camera(TiXmlElement *camera_element)
{
    Agi_camera camera;

    if (! camera.read_camera(camera_element, sensors_, file_version_))
    {
        return false;
    }

    cameras_.push_back(camera);

    return true;
}

//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
