//=============================================================================
#ifndef GRAPHENE_EYES_HELPER_H
#define GRAPHENE_EYES_HELPER_H
//=============================================================================

//== INCLUDES =================================================================


#include <vector>
#include <string>

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Point_set.h>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/utility/Agi_camera.h>

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>

#include <graphene/character/data_structure/Surface_mesh_skin.h>
#include <graphene/character/data_structure/Skeleton.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::geometry::Point_set;


//== CLASS DEFINITION =========================================================


class Eyes_helper
{
public:

    struct Mouse_callback_parameters
    {
        Eyes_helper* eyes_helper_;
        cv::Mat*     img_;
    };

public: //---------------------------------------------------- public functions

    // constructor
    Eyes_helper();

    bool load_frontal_camera(const std::string& filename_camera,
                             const std::string& filename_frontal_camera);

    bool load_contour_pixelcoords(const std::string& filename);


    /// pick eye contours;
    /// returns
    ///  1 if wrong numbers of points selected,
    ///  -1 when esc was pressed
    ///  0 for success
    int pick_contour_pixelcoords(const std::string& filename_frontal_photo);


    bool save_contour_pixelcoords(const std::string& filename);


    static void mouseCallback(int event, int x, int y, int flags, void *param)
    {
        Mouse_callback_parameters* mouse_callback_parameters = static_cast<Mouse_callback_parameters*>(param);

        Eyes_helper* self = static_cast<Eyes_helper*>(mouse_callback_parameters->eyes_helper_);
        cv::Mat&      img = *static_cast<cv::Mat*>(mouse_callback_parameters->img_);

        self->doMouseCallback(event, x, y, flags);

        if( event == CV_EVENT_LBUTTONDOWN )
        {
            cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0,255,0), -1);

            cv::imshow("frontal image (resized)", img);
        }
    }


    void doMouseCallback(int event, int x, int y, int flags)
    {
        if( event != CV_EVENT_LBUTTONDOWN )
        {
            return;
        }

        pixel_coords_eyecontour_.push_back( Vec2d( 3.0 * x, 3.0 * y ) );
    }


    bool fit_eyeball_proxies( const std::string& filename_photo,
                              Surface_mesh& mesh_left_eye,
                              Surface_mesh& mesh_right_eye,
                              const Point_set& orig_point_set,
                              const Surface_mesh& template_mesh,
                              const Mat4f& rigid_transformation ) const;


    bool update_eyes_icp( graphene::character::Surface_mesh_skin* left_eye_mesh,
                          graphene::character::Surface_mesh_skin* right_eye_mesh,
                          graphene::character::Surface_mesh_skin* base_mesh,
                          const std::string& filename_contour_left,
                          const std::string& filename_contour_right,
                          const std::string& filename_proj_left,
                          const std::string& filename_proj_right,
                          graphene::character::Skeleton& skeleton );


    bool update_eyes_ver2(const graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                          const graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                          const std::string& filename_selection_left,
                          const std::string& filename_selection_right,
                          const Vec3f& undeformed_left_eye_joint_pos,
                          const Vec3f& undeformed_right_eye_joint_pos,
                          graphene::character::Surface_mesh_skin* left_eye_mesh,
                          graphene::character::Surface_mesh_skin* right_eye_mesh,
                          graphene::character::Skeleton& skeleton);


    bool update_eyes_ver3(const graphene::surface_mesh::Surface_mesh& mesh_left_eye_proxy,
                          const graphene::surface_mesh::Surface_mesh& mesh_right_eye_proxy,
                          graphene::character::Surface_mesh_skin* left_eye_mesh,
                          graphene::character::Surface_mesh_skin* right_eye_mesh,
                          graphene::character::Skeleton& skeleton);


    bool compute_3d_contour_target_points( const Surface_mesh& mesh_left_eye,
                                           const Surface_mesh& mesh_right_eye,
                                           Surface_mesh& template_mesh,
                                           const Mat4f& rigid_transformation) const;


private:

    bool has_valid_eye_color(const Color& point_color, float min_brightness=0.35f, float max_white_deviation=0.1f, bool filter_skin = false) const;

    // in: src_left    points for left eye
    // in: dstc_left    points for left eye
    void register_spheres( const std::vector< Vec3f >& src_left,
                           const std::vector< Vec3f >& dst_left,
                           const std::vector< Vec3f >& src_right,
                           const std::vector< Vec3f >& dst_right,
                           graphene::Mat4f&  trans_left,
                           graphene::Mat4f&  trans_right,
                           double& relative_change ) const;

    void rgb_to_hsl(const Vec3f &rgb, Vec3f &hsl) const;

public:

    std::vector< Vec2d >   pixel_coords_eyecontour_;

    graphene::utility::Agi_camera agi_camera_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_EYES_HELPER_H
//=============================================================================
