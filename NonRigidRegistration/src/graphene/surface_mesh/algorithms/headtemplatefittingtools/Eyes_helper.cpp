//== INCLUDES ===================================================================


#include "Eyes_helper.h"

#include <fstream>

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/Triangle_kD_tree.h>

#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondence.h>

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>

//#include <graphene/surface_mesh/eigen_algorithms/rbf_deformer/RBF_deformer.h>

#include <graphene/geometry/Bounding_box_2D.h>
#include <graphene/geometry/registration.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdlib>


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::geometry::Bounding_box_2D;


//== IMPLEMENTATION ============================================================


Eyes_helper::
Eyes_helper()
{}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
load_frontal_camera(const std::string& filename_cameras,
                    const std::string& filename_frontal_camera)
{
    utility::Agi_cameras cams(filename_cameras);

    utility::Agi_camera* camera = cams.get_camera_by_filename(filename_frontal_camera);

    if (camera == nullptr)
    {
        std::cerr << "[ERROR] Unable to load frontal camera. Filenames:\n"
                  << "  - Calibration filename: \"" << filename_cameras << "\""
                  << "  - Camera filename: \"" << filename_frontal_camera<< "\""
                  << std::endl;

        return false;
    }

    agi_camera_ = *camera;

    return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
load_contour_pixelcoords(const std::string& filename)
{
    // load contour pixels
    std::ifstream ifs;
    ifs.open( filename.c_str() );
    if( ifs.is_open() != true )
    {
        std::cerr << "[WARNING] can't open 2d pixel-coordinates of eye contour from file: " << filename << std::endl;
        return false;
    }

    pixel_coords_eyecontour_.clear();

    float px, py;
    while (true)
    {
        ifs >> px >> py;
        if (ifs.eof()) break;
        Vec2d ff(px, py);
        pixel_coords_eyecontour_.push_back(ff);
    }

    ifs.close();
    return true;
}



//-----------------------------------------------------------------------------


int
Eyes_helper::
pick_contour_pixelcoords(const std::string& filename_frontal_photo)
{
    // load image
    cv::Mat frontal_image = cv::imread( filename_frontal_photo.c_str() );
    if(!frontal_image.data)
    {
        std::cerr << "[ERROR] Can't load frontal photo from file: " << filename_frontal_photo << std::endl;
        return false;
    }

    const double rows = frontal_image.rows;
    const double cols = frontal_image.cols;

    cv::Size size_small(cols/3.0, rows/3.0); // TODO
    cv::Mat frontal_image_small;
    cv::resize(frontal_image, frontal_image_small, size_small); //resize image

    pixel_coords_eyecontour_.clear();

    cv::namedWindow("frontal image (resized)");
    cv::moveWindow("frontal image (resized)", 10,10);


    // set mouse callback
    Mouse_callback_parameters mouse_callback_parameters;
    mouse_callback_parameters.eyes_helper_ = this;
    mouse_callback_parameters.img_ = &frontal_image_small;
//    cv::setMouseCallback( "frontal image (resized)", mouseCallback, this );
    cv::setMouseCallback( "frontal image (resized)", mouseCallback, &mouse_callback_parameters );

    // show the image
    cv::imshow("frontal image (resized)", frontal_image_small);

    int key=0;
    while ( true)
    {
        key = cv::waitKey(0);

        if (key == 27)
        {
            cv::destroyWindow("frontal image (resized)");
            return -1;
        }
        else if (key == ' ')
        {
            if ( ! (pixel_coords_eyecontour_.size() == 14 || pixel_coords_eyecontour_.size() == 12) )
            {
                std::cerr << "[ERROR] Invalid number of 2D contour points: " << pixel_coords_eyecontour_.size() << ". Clear..." << std::endl;
                pixel_coords_eyecontour_.clear();
                return 1;
            }
            else
            {
                cv::destroyWindow("frontal image (resized)");
                return 0;
            }
        }
    }


}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
save_contour_pixelcoords(const std::string& filename)
{
    if (pixel_coords_eyecontour_.size() == 0)
    {
        std::cerr << "[ERROR] No 2d pixel-coordinates available." << std::endl;
        return false;
    }

    std::ofstream ofs( filename.c_str() );
    if( ofs.is_open() != true )
    {
        std::cerr << "[ERROR] Can't open 2d pixel-coordinates of eye contour from file: " << filename << std::endl;
        return false;
    }

    for (unsigned int i(0); i < pixel_coords_eyecontour_.size(); i++)
    {
        ofs << pixel_coords_eyecontour_[i][0];
        ofs << " ";
        ofs << pixel_coords_eyecontour_[i][1];
        ofs << std::endl;
    }
    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
fit_eyeball_proxies(const std::string& filename_photo,
                     Surface_mesh& mesh_left_eye,
                     Surface_mesh& mesh_right_eye,
                     const Point_set& orig_point_set,
                     const Surface_mesh &template_mesh,
                     const Mat4f& rigid_transformation) const
{
    if ( !agi_camera_.is_initialized() )
    {
        std::cerr << "[ERROR] cannot fit eyeball proxfies. Agi_camera is not inizialized!" << std::endl;
        return false;
    }


    // open image
    const cv::Mat frontal_face_img = cv::imread( filename_photo.c_str() );
    if(!frontal_face_img.data)
    {
        std::cout << "[ERROR] Can't load frontal photo." << std::endl;
        return false;
    }
    const double rows = frontal_face_img.rows;
    const double cols = frontal_face_img.cols;

    const size_t coords_div_2 = pixel_coords_eyecontour_.size() / 2;

    // compute 2d bounding box of left eye contour points
    Bounding_box_2D bbox_left_eye_sclera_pixels;
    for (size_t pix_id = coords_div_2; pix_id < pixel_coords_eyecontour_.size(); ++pix_id)
    {
        Pixel_coordinate p( pixel_coords_eyecontour_[pix_id][0], pixel_coords_eyecontour_[pix_id][1] );
        bbox_left_eye_sclera_pixels += p;
    }
    const double bbox2d_left_min_x = 0.98f*bbox_left_eye_sclera_pixels.min()[0];
    const double bbox2d_left_min_y = 0.98f*bbox_left_eye_sclera_pixels.min()[1];
    const double bbox2d_left_max_x = 1.02f*bbox_left_eye_sclera_pixels.max()[0];
    const double bbox2d_left_max_y = 1.02f*bbox_left_eye_sclera_pixels.max()[1];


    // compute 2d bounding box of right eye contour points
    Bounding_box_2D bbox_right_eye_sclera_pixels;
    for (size_t pix_id = 0; pix_id < coords_div_2; ++pix_id)
    {
        Pixel_coordinate p( pixel_coords_eyecontour_[pix_id][0], pixel_coords_eyecontour_[pix_id][1] );
        bbox_right_eye_sclera_pixels += p;
    }
    const double bbox2d_right_min_x = 0.98f*bbox_right_eye_sclera_pixels.min()[0];
    const double bbox2d_right_min_y = 0.98f*bbox_right_eye_sclera_pixels.min()[1];
    const double bbox2d_right_max_x = 1.02f*bbox_right_eye_sclera_pixels.max()[0];
    const double bbox2d_right_max_y = 1.02f*bbox_right_eye_sclera_pixels.max()[1];


    // get region-of-interest left eye
    unsigned int faceImgROIcols_left = (unsigned int) (bbox2d_left_max_x-bbox2d_left_min_x);
    unsigned int faceImgROIrows_left = (unsigned int) (bbox2d_left_max_y-bbox2d_left_min_y);
    // cv::Mat faceImgROI_left = frontal_face_img( cv::Rect( bbox2d_left_min_x, bbox2d_left_min_y, faceImgROIcols_left, faceImgROIrows_left ) );
    // cv::namedWindow("test_left");
    // cv::imshow("test_left", faceImgROI_left);
    // cv::waitKey();


    // get region-of-interest right eye
    unsigned int faceImgROIcols_right  = (unsigned int) (bbox2d_right_max_x-bbox2d_right_min_x);
    unsigned int faceImgROIrows_right  = (unsigned int) (bbox2d_right_max_y-bbox2d_right_min_y);
    // cv::Mat faceImgROI_right = frontal_face_img( cv::Rect( bbox2d_right_min_x, bbox2d_right_min_y, faceImgROIcols_right, faceImgROIrows_right ) );
    // cv::namedWindow("test_right");
    // cv::imshow("test_right", faceImgROI_right);
    // cv::waitKey();


    // filter pixels left eye
    cv::Mat faceImgMask_left(rows, cols, CV_8UC1);
    faceImgMask_left = cv::Scalar(0);
    for ( unsigned int i = (unsigned int) bbox2d_left_min_y; i < (unsigned int) (bbox2d_left_min_y + faceImgROIrows_left); ++i )
    {
        for ( unsigned int j = (unsigned int) bbox2d_left_min_x; j < (unsigned int) (bbox2d_left_min_x + faceImgROIcols_left); ++j )
        {
            faceImgMask_left.at<uchar>(i,j) = 255;
        }
    }


    // filter pixels right eye
    cv::Mat faceImgMask_right(rows, cols, CV_8UC1);
    faceImgMask_right = cv::Scalar(0);
    for ( unsigned int i = (unsigned int) bbox2d_right_min_y; i < (unsigned int) (bbox2d_right_min_y + faceImgROIrows_right); ++i )
    {
        for ( unsigned int j = (unsigned int) bbox2d_right_min_x; j < (unsigned int) (bbox2d_right_min_x + faceImgROIcols_right); ++j )
        {
            faceImgMask_right.at<uchar>(i,j) = 255;
        }
    }


    // Close the image
    // cv::Mat element(10,10,CV_8U,cv::Scalar(1));
    // cv::Mat closed_left;
    // cv::morphologyEx(faceImgMask_left, closed_left, cv::MORPH_CLOSE, element);
    // faceImgMask_left = closed_left;
    // cv::Mat closed_right;
    // cv::morphologyEx(faceImgMask_right, closed_right, cv::MORPH_CLOSE, element);
    // faceImgMask_right = closed_right;


    //cv::Mat faceImgMaskROI_left = faceImgMask_left( cv::Rect( corrected_x_min, bbox2d_left_min_y, faceImgROIcols_left, faceImgROIrows_left ) );
    // cv::namedWindow("faceImgMaskROI_left");
    // cv::imshow("faceImgMaskROI_left", faceImgMaskROI_left);
    // cv::waitKey();


    //cv::Mat faceImgMaskROI_right = faceImgMask_right( cv::Rect( bbox2d_right_min_x, bbox2d_right_min_y, faceImgROIcols_right, faceImgROIrows_right ) );
    // cv::namedWindow("faceImgMaskROI_right");
    // cv::imshow("faceImgMaskROI_right", faceImgMaskROI_right);
    // cv::waitKey();


    // collect 3D points of sclera pixels
    Point_set point_set_left_eye;
    Point_set point_set_right_eye;
    for (unsigned int i = 0; i < orig_point_set.points_.size(); ++i)
    {
        const Vec2d point_ps_on_imageplane = agi_camera_.world_to_pixel( orig_point_set.points_[i] );
        unsigned int pix_proj_x = point_ps_on_imageplane[0];
        unsigned int pix_proj_y = point_ps_on_imageplane[1];

        if ( (pix_proj_y >= 0) && (pix_proj_y < rows) &&
             (pix_proj_x >= 0) && (pix_proj_x < cols) &&
             (faceImgMask_left.at<uchar>( pix_proj_y, pix_proj_x ) > 0) ) // TODO evtl. auch Nachbarn anschauen ?!
        {
            Color c;
            c[2] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[0] / 255.0;
            c[1] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[1] / 255.0;
            c[0] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[2] / 255.0;
            if (has_valid_eye_color(c, 0.35f, 0.1f, false))
            {
                point_set_left_eye.points_.push_back(orig_point_set.points_[i]);
                point_set_left_eye.normals_.push_back(orig_point_set.normals_[i]);
                if (orig_point_set.colors_.size() > 0)
                    point_set_left_eye.colors_.push_back(orig_point_set.colors_[i]);
            }
        }

        if ( (pix_proj_y >= 0) && (pix_proj_y < rows) &&
             (pix_proj_x >= 0) && (pix_proj_x < cols) &&
             (faceImgMask_right.at<uchar>( pix_proj_y, pix_proj_x ) > 0) ) // TODO evtl. auch Nachbarn anschauen ?!
        {
            Color c;
            c[2] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[0] / 255.0;
            c[1] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[1] / 255.0;
            c[0] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[2] / 255.0;
            if (has_valid_eye_color(c, 0.35f, 0.1f, false))
            {
                point_set_right_eye.points_.push_back(orig_point_set.points_[i]);
                point_set_right_eye.normals_.push_back(orig_point_set.normals_[i]);
                if (orig_point_set.colors_.size() > 0)
                    point_set_right_eye.colors_.push_back(orig_point_set.colors_[i]);
            }
        }
    }



    if ( (point_set_left_eye.points().size()  < 20) ||
         (point_set_right_eye.points().size() < 20) )
    {
        std::cout << "Eyes_helper::fit_eyeball_proxies: [WARNING] Insufficient sclera points found. Trying again with less strict filter." << std::endl;

        point_set_left_eye.clear();
        point_set_right_eye.clear();

        for (unsigned int i = 0; i < orig_point_set.points_.size(); ++i)
        {
            const Vec2d point_ps_on_imageplane = agi_camera_.world_to_pixel( orig_point_set.points_[i] );
            unsigned int pix_proj_x = point_ps_on_imageplane[0];
            unsigned int pix_proj_y = point_ps_on_imageplane[1];

            if ( (pix_proj_y >= 0) && (pix_proj_y < rows) &&
                 (pix_proj_x >= 0) && (pix_proj_x < cols) &&
                 (faceImgMask_left.at<uchar>( pix_proj_y, pix_proj_x ) > 0) ) // TODO evtl. auch Nachbarn anschauen ?!
            {
                Color c;
                c[2] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[0] / 255.0;
                c[1] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[1] / 255.0;
                c[0] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[2] / 255.0;
                if (has_valid_eye_color(c, 0.25f, 0.2f,false))
                {
                    point_set_left_eye.points_.push_back(orig_point_set.points_[i]);
                    point_set_left_eye.normals_.push_back(orig_point_set.normals_[i]);
                    if (orig_point_set.colors_.size() > 0)
                        point_set_left_eye.colors_.push_back(orig_point_set.colors_[i]);
                }
            }

            if ( (pix_proj_y >= 0) && (pix_proj_y < rows) &&
                 (pix_proj_x >= 0) && (pix_proj_x < cols) &&
                 (faceImgMask_right.at<uchar>( pix_proj_y, pix_proj_x ) > 0) ) // TODO evtl. auch Nachbarn anschauen ?!
            {
                Color c;
                c[2] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[0] / 255.0;
                c[1] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[1] / 255.0;
                c[0] = frontal_face_img.at<cv::Vec3b>(pix_proj_y,pix_proj_x)[2] / 255.0;
                if (has_valid_eye_color(c, 0.25f, 0.2f,false))
                {
                    point_set_right_eye.points_.push_back(orig_point_set.points_[i]);
                    point_set_right_eye.normals_.push_back(orig_point_set.normals_[i]);
                    if (orig_point_set.colors_.size() > 0)
                        point_set_right_eye.colors_.push_back(orig_point_set.colors_[i]);
                }
            }
        }
    }

    if (point_set_left_eye.points().size() < 20 ||
        point_set_right_eye.points().size() < 20)
    {
        std::cerr << "[ERROR] Still not enough sclera points. Aborting eyelid fitting." << std::endl;
        return false;
    }


    for (unsigned int i = 0; i < point_set_left_eye.size(); ++ i)
    {
        Point p = point_set_left_eye.points_[i];
        p = affine_transform(rigid_transformation, p);
        point_set_left_eye.points_[i] = p;
    }
    for (unsigned int i = 0; i < point_set_right_eye.size(); ++ i)
    {
        Point p = point_set_right_eye.points_[i];
        p = affine_transform(rigid_transformation, p);
        point_set_right_eye.points_[i] = p;
    }
    // DEBUG
    // std::cerr << "[DEBUG] WRITE PS LEFT EYE AND RIGHT EYE - A" << std::endl;
    // std::cerr << "[DEBUG] point_set_left_eye.points_.size(): " << point_set_left_eye.points_.size() << std::endl;
    // std::cerr << "[DEBUG] point_set_right_eye.points_.size(): " << point_set_right_eye.points_.size() << std::endl;
    // point_set_left_eye.write("DEBUG_PS_LEFT_EYE.xyz");
    // point_set_right_eye.write("DEBUG_PS_RIGHT_EYE.xyz");
    // std::cerr << "[DEBUG] WRITE PS LEFT EYE AND RIGHT EYE - B" << std::endl;



    //compute bbox and center of left eye sclera points
    Vec3f avg_sclera_points_normal(0.0f);
    Vec3f avg_sclera_point(0.0f);
    for (size_t i=0; i < point_set_left_eye.points().size(); ++i)
    {
        avg_sclera_point         += point_set_left_eye.points()[i];
        avg_sclera_points_normal += point_set_left_eye.normals_[i];
    }
    avg_sclera_point         /= point_set_left_eye.points_.size();
    avg_sclera_points_normal /= point_set_left_eye.normals_.size();
    avg_sclera_points_normal.normalize();

    // initial barycenter of left eye proxy
    geometry::Bounding_box bbox_left_eye_mesh;
    for (auto v : mesh_left_eye.vertices())
    {
        bbox_left_eye_mesh += mesh_left_eye.position(v);
    }

    //transform mesh to sclera points
    const Vec3f translate_lefteye_z = - 0.25f * bbox_left_eye_mesh.size() * avg_sclera_points_normal;
    const Vec3f diff_left_eye = (avg_sclera_point - bbox_left_eye_mesh.center());
    const Mat4f initial_trans_left_eye = Mat4f::translate(diff_left_eye + translate_lefteye_z);
    transform_mesh(mesh_left_eye, initial_trans_left_eye);




    //compute bbox and center of right eye sclera points
    avg_sclera_points_normal = Vec3f(0.0f);
    avg_sclera_point         = Vec3f(0.0f);
    for (size_t i=0; i < point_set_right_eye.points().size(); ++i)
    {
        avg_sclera_point         += point_set_right_eye.points()[i];
        avg_sclera_points_normal += point_set_right_eye.normals_[i];
    }
    avg_sclera_point         /= point_set_right_eye.points_.size();
    avg_sclera_points_normal /= point_set_right_eye.normals_.size();
    avg_sclera_points_normal.normalize();

    // initial barycenter of right eye proxy
    geometry::Bounding_box bbox_right_eye_mesh;
    for (auto v : mesh_right_eye.vertices())
    {
        bbox_right_eye_mesh += mesh_right_eye.position(v);
    }

    //transform mesh to sclera points
    const Vec3f translate_righteye_z = - 0.25f * bbox_right_eye_mesh.size() * avg_sclera_points_normal;
    const Vec3f diff_right_eye = (avg_sclera_point - bbox_right_eye_mesh.center());
    const Mat4f initial_trans_right_eye = Mat4f::translate(diff_right_eye + translate_righteye_z);
    transform_mesh(mesh_right_eye, initial_trans_right_eye);


    Correspondences_settings cset;
    cset.dist_until = 0.05;
    cset.filter_dist = true;
    cset.constr_dir = corresp_dir_ps2mesh;
    cset.normal_until = 80.0;
    cset.filter_normals = true;

    std::vector<Correspondence> correspondences_left;
    std::vector<Correspondence> correspondences_right;

    // iteratively fit sphere to sclera points by adjusting positions and coupling (identical) spheres
    bool converged = false;
    do
    {

        compute_correspondences(&mesh_left_eye, &point_set_left_eye, cset, correspondences_left);
        compute_correspondences(&mesh_right_eye, &point_set_right_eye, cset, correspondences_right);


        if (correspondences_left.empty() || correspondences_right.empty())
        {
            std::cerr << "Eyes_helper::fit_eyeball_proxies: [ERROR] Unable to find correspondences." << std::endl;
            return false;
        }

        // transform
        const unsigned int nc_left = correspondences_left.size();
        std::vector<Point> p_valid_left(nc_left), q_valid_left(nc_left);
        for (unsigned int i = 0; i < nc_left; ++i)
        {
            p_valid_left[i] = correspondences_left[i].on_template;
            q_valid_left[i] = correspondences_left[i].on_ps;        
        }
        const unsigned int nc_right = correspondences_right.size();
        std::vector<Point> p_valid_right(nc_right), q_valid_right(nc_right);
        for (unsigned int i = 0; i < nc_right; ++i)
        {
            p_valid_right[i] = correspondences_right[i].on_template;
            q_valid_right[i] = correspondences_right[i].on_ps;        
        }
        double relative_change;

        Mat4f M_left;
        Mat4f M_right;

        register_spheres( p_valid_left, q_valid_left,
                          p_valid_right, q_valid_right,
                          M_left,
                          M_right,
                          relative_change );

        /*
        M_left  = geometry::registration(p_valid_left, q_valid_left, geometry::RIGID_REGISTRATION, &relative_change);
        M_right = geometry::registration(p_valid_right, q_valid_right, geometry::RIGID_REGISTRATION, &relative_change);
        */

        transform_mesh(mesh_left_eye, M_left);
        transform_mesh(mesh_right_eye, M_right);

        std::cout << "Eyes_helper::fit_eyeball_proxies: [STATUS] Relative change of eye ball fitting: " << relative_change << std::endl;

        if (relative_change < 0.005)
        {
            converged = true;
        }

    }
    while ( !converged );
/*
    mesh_left_eye.write("test1.off");
    mesh_right_eye.write("test2.off");

    point_set_left_eye.write_cxyz("test1.cxyz");
    point_set_right_eye.write_cxyz("test2.cxyz");
*/
    return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
update_eyes_icp( graphene::character::Surface_mesh_skin* left_eye_mesh,
                 graphene::character::Surface_mesh_skin* right_eye_mesh,
                 graphene::character::Surface_mesh_skin* base_mesh,
                 const std::string& filename_contour_left,
                 const std::string& filename_contour_right,
                 const std::string& filename_proj_left,
                 const std::string& filename_proj_right,
                 graphene::character::Skeleton& skeleton )
{
    if (left_eye_mesh && right_eye_mesh)
    {
        // contour points -> both eyes
        Point_set contour_template_both_eyes;

        // contour points -> left eye
        Point_set contour_template_left_eye;
        auto sel_contour_prop_left = base_mesh->vertex_property<bool>("v:sel_contour_left", false);
        {
            std::ifstream ifs_contour_left( filename_contour_left.c_str() );
            unsigned int handle_contour_left;
            while (true)
            {
                ifs_contour_left >> handle_contour_left;
                if (ifs_contour_left.eof()) break;
                sel_contour_prop_left[Surface_mesh::Vertex(handle_contour_left)] = true;
            }
            ifs_contour_left.close();

            for (auto v : base_mesh->vertices())
            {
                if (sel_contour_prop_left[v])
                {
                    contour_template_left_eye.points_.push_back( base_mesh->position(v) );
                    contour_template_both_eyes.points_.push_back( base_mesh->position(v) );
                }
            }
        }

        // contour points -> right eye
        Point_set contour_template_right_eye;
        auto sel_contour_prop_right = base_mesh->vertex_property<bool>("v:sel_contour_right", false);
        {
            std::ifstream ifs_contour_right( filename_contour_right.c_str() );
            unsigned int handle_contour_right;
            while (true)
            {
                ifs_contour_right >> handle_contour_right;
                if (ifs_contour_right.eof()) break;
                sel_contour_prop_right[Surface_mesh::Vertex(handle_contour_right)] = true;
            }
            ifs_contour_right.close();

            for (auto v : base_mesh->vertices())
            {
                if (sel_contour_prop_right[v])
                {
                    contour_template_right_eye.points_.push_back( base_mesh->position(v) );
                    contour_template_both_eyes.points_.push_back( base_mesh->position(v) );
                }
            }
        }

        // load selection with points of projection -> left eye
        auto sel_proj_prop_left = base_mesh->vertex_property<bool>("v:sel_proj_left", false);
        {
            std::ifstream ifs_proj_left( filename_proj_left.c_str() );
            unsigned int handle_proj_left;
            while (true)
            {
                ifs_proj_left >> handle_proj_left;
                if (ifs_proj_left.eof()) break;
                sel_proj_prop_left[Surface_mesh::Vertex(handle_proj_left)] = true;
            }
            ifs_proj_left.close();
        }

        // load selection with points of projection -> right eye
        auto sel_proj_prop_right = base_mesh->vertex_property<bool>("v:sel_proj_right", false);
        {
            std::ifstream ifs_proj_right( filename_proj_right.c_str() );
            unsigned int handle_proj_right;
            while (true)
            {
                ifs_proj_right >> handle_proj_right;
                if (ifs_proj_right.eof()) break;
                sel_proj_prop_right[Surface_mesh::Vertex(handle_proj_right)] = true;
            }
            ifs_proj_right.close();
        }

        // barycenter of contour points -> left eye
        Point bary_contour_points_left(0,0,0);
        {
            if (0 == contour_template_left_eye.points_.size())
            {
                std::cerr << "[ERROR] 0 == contour_template_left_eye.points_.size()." << std::endl;
                return false;
            }

            for (unsigned int i = 0; i < contour_template_left_eye.points_.size(); ++i)
            {
                bary_contour_points_left += contour_template_left_eye.points_[i];
            }
            bary_contour_points_left /= contour_template_left_eye.points_.size();
        }

        // barycenter of contour points -> right eye
        Point bary_contour_points_right(0,0,0);
        {
            if (0 == contour_template_right_eye.points_.size())
            {
                std::cerr << "[ERROR] 0 == contour_template_right_eye.points_.size()." << std::endl;
                return false;
            }

            for (unsigned int i = 0; i < contour_template_right_eye.points_.size(); ++i)
            {
                bary_contour_points_right += contour_template_right_eye.points_[i];
            }
            bary_contour_points_right /= contour_template_right_eye.points_.size();
        }

        // barycenter of contour points -> both eyes
        Point bary_contour_points_both(0,0,0);
        {
            if (0 == contour_template_both_eyes.points_.size())
            {
                std::cerr << "[ERROR] 0 == contour_template_both_eyes.points_.size()." << std::endl;
                return false;
            }

            for (unsigned int i = 0; i < contour_template_both_eyes.points_.size(); ++i)
            {
                bary_contour_points_both += contour_template_both_eyes.points_[i];
            }
            bary_contour_points_both /= contour_template_both_eyes.points_.size();
        }

        // desired height for eyes
        const double desired_eyes_height = bary_contour_points_both[1]; // ATTENTION: the normal is: (0,1,0)

        // fit eyes

        // update left eye mesh
        Point bary_left_eye_before;  // barycenter of left eye before optimization
        Point bary_left_eye_after;   // barycenter of left eye after optimization
        Vec3f viewing_dir_left_eye = Vec3f(0, 0, 1);
        {
            // barycenter of left eye before optimization
            bary_left_eye_before = Point(0,0,0);
            for (auto v : left_eye_mesh->vertices())
            {
                bary_left_eye_before += left_eye_mesh->position(v);
            }
            bary_left_eye_before /= left_eye_mesh->n_vertices();

            const unsigned int max_iters_left_eye = 15;
            unsigned int iter_left_eye = 0;
            bool converged_left = false;
            do
            {
                iter_left_eye++;

                // kd tree of eye mesh
                graphene::surface_mesh::Triangle_kD_tree target_kd_left(*left_eye_mesh);

                std::vector< Point > source_points_left;
                std::vector< Point > target_points_left;
                for (auto v : base_mesh->vertices())
                {
//                    if (sel_proj_prop_left[v])
                    if (sel_contour_prop_left[v])
                    {
                        target_points_left.push_back( base_mesh->position(v) );

                        const auto nn_tri = target_kd_left.nearest( base_mesh->position(v) );
                        source_points_left.push_back( nn_tri.nearest );
                    }
                }

                // rigid transformation -----> OPTIMIZE TRANSLATION
                double relative_change_left;
                Mat4f M_left = graphene::geometry::registration(source_points_left, target_points_left, graphene::geometry::RIGID_REGISTRATION, &relative_change_left, true); // true means no rotation

                // update height, s.t. eyes have the same (claimed) height
                Point bary_left_eye(0,0,0);
                for (auto v : left_eye_mesh->vertices())
                {
                    bary_left_eye += left_eye_mesh->position(v);
                }
                bary_left_eye /= left_eye_mesh->n_vertices();
                M_left(1, 3) = desired_eyes_height - bary_left_eye[1];
                transform_mesh(*left_eye_mesh, M_left);

                if (relative_change_left < 0.01)
                {
                    converged_left = true;
                }

                // rigid transformation -----> OPTIMIZE ROTATION (viewing direction)
                bary_left_eye = Point(0,0,0);
                for (auto v : left_eye_mesh->vertices())
                {
                    bary_left_eye += left_eye_mesh->position(v);
                }
                bary_left_eye /= left_eye_mesh->n_vertices();

                const Vec3f  temp_viewing_dir_left_eye = (bary_contour_points_left - bary_left_eye).normalize();
                const double cos_angle = dot(viewing_dir_left_eye, temp_viewing_dir_left_eye);
                const double angle = acos( cos_angle ) * 180.0 / M_PI;
                const Vec3f  axis = cross(viewing_dir_left_eye, temp_viewing_dir_left_eye);
                const Mat4f  M_rotate = Mat4f::rotate(axis, angle);

                // rotate eye
                for (auto v : left_eye_mesh->vertices())
                {
                    left_eye_mesh->position(v) -= bary_left_eye;
                }
                transform_mesh(*left_eye_mesh, M_rotate);
                for (auto v : left_eye_mesh->vertices())
                {
                    left_eye_mesh->position(v) += bary_left_eye;
                }

                // bary left eye after
                bary_left_eye = Point(0,0,0);
                for (auto v : left_eye_mesh->vertices())
                {
                    bary_left_eye += left_eye_mesh->position(v);
                }
                bary_left_eye /= left_eye_mesh->n_vertices();
                bary_left_eye_after = bary_left_eye;

                // transform viewing direction viewing_dir_left_eye
                viewing_dir_left_eye = Mat3f(M_rotate) * viewing_dir_left_eye;
            }
            while ( !converged_left && iter_left_eye < max_iters_left_eye);

            for (auto v : left_eye_mesh->vertices())
            {
                left_eye_mesh->position(v) -= 0.0025 * viewing_dir_left_eye; // slightly move eye behind
            }
        }

        // update right eye mesh
        Point bary_right_eye_before;  // barycenter of right eye before optimization
        Point bary_right_eye_after;   // barycenter of right eye after optimization
        Vec3f viewing_dir_right_eye = Vec3f(0, 0, 1);
        {
            // barycenter of right eye before optimization
            bary_right_eye_before = Point(0,0,0);
            for (auto v : right_eye_mesh->vertices())
            {
                bary_right_eye_before += right_eye_mesh->position(v);
            }
            bary_right_eye_before /= right_eye_mesh->n_vertices();

            const unsigned int max_iters_right_eye = 15;
            unsigned int iter_right_eye = 0;
            bool converged_right = false;
            do
            {
                iter_right_eye++;

                // kd tree of eye mesh
                graphene::surface_mesh::Triangle_kD_tree target_kd_right(*right_eye_mesh);

                std::vector< Point > source_points_right;
                std::vector< Point > target_points_right;
                for (auto v : base_mesh->vertices())
                {
                    if (sel_contour_prop_right[v])
//                    if (sel_proj_prop_right[v])
                    {
                        target_points_right.push_back( base_mesh->position(v) );

                        const auto nn_tri = target_kd_right.nearest( base_mesh->position(v) );
                        source_points_right.push_back( nn_tri.nearest );
                    }
                }

                // rigid transformation -----> OPTIMIZE TRANSLATION
                double relative_change_right;
                Mat4f M_right = graphene::geometry::registration(source_points_right, target_points_right, graphene::geometry::RIGID_REGISTRATION, &relative_change_right, true); // true means no rotation

                // update height, s.t. eyes have the same (claimed) height
                Point bary_right_eye(0,0,0);
                for (auto v : right_eye_mesh->vertices())
                {
                    bary_right_eye += right_eye_mesh->position(v);
                }
                bary_right_eye /= right_eye_mesh->n_vertices();
                M_right(1, 3) = desired_eyes_height - bary_right_eye[1];
                transform_mesh(*right_eye_mesh, M_right);

                if (relative_change_right < 0.01)
                {
                    converged_right = true;
                }

                // rigid transformation -----> OPTIMIZE ROTATION (viewing direction)
                bary_right_eye = Point(0,0,0);
                for (auto v : right_eye_mesh->vertices())
                {
                    bary_right_eye += right_eye_mesh->position(v);
                }
                bary_right_eye /= right_eye_mesh->n_vertices();

                const Vec3f  temp_viewing_dir_right_eye = (bary_contour_points_right - bary_right_eye).normalize();
                const double cos_angle = dot(viewing_dir_right_eye, temp_viewing_dir_right_eye);
                const double angle = acos( cos_angle ) * 180.0 / M_PI;
                const Vec3f  axis = cross(viewing_dir_right_eye, temp_viewing_dir_right_eye);
                const Mat4f  M_rotate = Mat4f::rotate(axis, angle);

                // rotate eye
                for (auto v : right_eye_mesh->vertices())
                {
                    right_eye_mesh->position(v) -= bary_right_eye;
                }
                transform_mesh(*right_eye_mesh, M_rotate);
                for (auto v : right_eye_mesh->vertices())
                {
                    right_eye_mesh->position(v) += bary_right_eye;
                }

                // bary right eye after
                bary_right_eye = Point(0,0,0);
                for (auto v : right_eye_mesh->vertices())
                {
                    bary_right_eye += right_eye_mesh->position(v);
                }
                bary_right_eye /= right_eye_mesh->n_vertices();
                bary_right_eye_after = bary_right_eye;

                // transform viewing direction viewing_dir_right_eye
                viewing_dir_right_eye = Mat3f(M_rotate) * viewing_dir_right_eye;
            }
            while ( !converged_right && iter_right_eye < max_iters_right_eye);

            for (auto v : right_eye_mesh->vertices())
            {
                right_eye_mesh->position(v) -= 0.0025 * viewing_dir_right_eye; // slightly move eye behind
            }
        }

        // adjust rotation of eye meshes
        const Point  distant_point_left    = bary_left_eye_after  + 100.0 * viewing_dir_left_eye;
        const Point  distant_point_right   = bary_right_eye_after + 100.0 * viewing_dir_right_eye;
        const Point  distant_point_mid     = (distant_point_left + distant_point_right) / 2.0;

        const Vec3f  good_viewingdir_left  = (distant_point_mid - bary_left_eye_after).normalize();
        const Vec3f  good_viewingdir_right = (distant_point_mid - bary_right_eye_after).normalize();

        const double cos_angle_good_left   = dot(viewing_dir_left_eye, good_viewingdir_left);
        const double angle_good_left       = acos( cos_angle_good_left ) * 180.0 / M_PI;
        const Vec3f  axis_good_left        = cross(viewing_dir_left_eye, good_viewingdir_left);
        const Mat4f  M_rotate_good_left    = Mat4f::rotate(axis_good_left, angle_good_left);

        const double cos_angle_good_right  = dot(viewing_dir_right_eye, good_viewingdir_right);
        const double angle_good_right      = acos( cos_angle_good_right ) * 180.0 / M_PI;
        const Vec3f  axis_good_right       = cross(viewing_dir_right_eye, good_viewingdir_right);
        const Mat4f  M_rotate_good_right   = Mat4f::rotate(axis_good_right, angle_good_right);

        for (auto v : left_eye_mesh->vertices())
        {
            left_eye_mesh->position(v) -= bary_left_eye_after;
        }
        transform_mesh(*left_eye_mesh, M_rotate_good_left);
        for (auto v : left_eye_mesh->vertices())
        {
            left_eye_mesh->position(v) += bary_left_eye_after;
        }

        for (auto v : right_eye_mesh->vertices())
        {
            right_eye_mesh->position(v) -= bary_right_eye_after;
        }
        transform_mesh(*right_eye_mesh, M_rotate_good_right);
        for (auto v : right_eye_mesh->vertices())
        {
            right_eye_mesh->position(v) += bary_right_eye_after;
        }

        left_eye_mesh->update_face_normals();
        left_eye_mesh->update_vertex_normals();
        right_eye_mesh->update_face_normals();
        right_eye_mesh->update_vertex_normals();


        // translate eye joints -----------------------------------------------

            //skeleton.align_axes();

        character::Joint* left_eye_joint  = skeleton.get_joint("LeftEye");
        character::Joint* left_eye_joint_parent = left_eye_joint->parent_;
        character::Joint* right_eye_joint = skeleton.get_joint("RightEye");
        character::Joint* right_eye_joint_parent = right_eye_joint->parent_;

        const Vec3f offset_r = bary_right_eye_after - right_eye_joint_parent->get_global_translation();
        const Vec3f offset_l = bary_left_eye_after  - left_eye_joint_parent->get_global_translation();

        left_eye_joint->local_ = Mat4f::identity();
        left_eye_joint->local_[12] = offset_l[0];
        left_eye_joint->local_[13] = offset_l[1];
        left_eye_joint->local_[14] = offset_l[2];
        left_eye_joint->bind_pose_local_ = left_eye_joint->local_;

        right_eye_joint->local_ = Mat4f::identity();
        right_eye_joint->local_[12] = offset_r[0];
        right_eye_joint->local_[13] = offset_r[1];
        right_eye_joint->local_[14] = offset_r[2];
        right_eye_joint->bind_pose_local_ = right_eye_joint->local_;

        skeleton.init();

        return true;
    }
    else
    {
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
update_eyes_ver2(const graphene::surface_mesh::Surface_mesh& mesh_source_undeformed,
                 const graphene::surface_mesh::Surface_mesh& mesh_target_undeformed,
                 const std::string& filename_selection_left,
                 const std::string& filename_selection_right,
                 const Vec3f& undeformed_left_eye_joint_pos,
                 const Vec3f& undeformed_right_eye_joint_pos,
                 graphene::character::Surface_mesh_skin* left_eye_mesh,
                 graphene::character::Surface_mesh_skin* right_eye_mesh,
                 graphene::character::Skeleton& skeleton)
{
    std::vector<unsigned int> selection_left_vec;
    if (!graphene::surface_mesh::load_selection_from_file(selection_left_vec, filename_selection_left))
    {
        std::cerr << "[ERROR] Can't open selection (left eye) to update eye. Filename: " << filename_selection_left << std::endl;
        return false;
    }
    std::vector<unsigned int> selection_right_vec;
    if (!graphene::surface_mesh::load_selection_from_file(selection_right_vec, filename_selection_right))
    {
        std::cerr << "[ERROR] Can't open selection (right eye) to update eye. Filename: " << filename_selection_right << std::endl;
        return false;
    }


    std::vector<Point> src_points_left;
    std::vector<Point> tar_points_left;
    std::vector<Point> src_points_right;
    std::vector<Point> tar_points_right;
    // std::vector<Point> src_points_both;
    // std::vector<Point> tar_points_both;
    for ( unsigned int i = 0; i < selection_left_vec.size(); ++i)
    {
        Surface_mesh::Vertex v(selection_left_vec[i]);

        src_points_left.push_back(mesh_source_undeformed.position(v));
        tar_points_left.push_back(mesh_target_undeformed.position(v));

        // src_points_both.push_back(mesh_source_undeformed.position(v));
        // tar_points_both.push_back(mesh_target_undeformed.position(v));
    }
    for ( unsigned int i = 0; i < selection_right_vec.size(); ++i)
    {
        Surface_mesh::Vertex v(selection_right_vec[i]);

        src_points_right.push_back(mesh_source_undeformed.position(v));
        tar_points_right.push_back(mesh_target_undeformed.position(v));

        // src_points_both.push_back(mesh_source_undeformed.position(v));
        // tar_points_both.push_back(mesh_target_undeformed.position(v));
    }

    // rbf deformation

    // graphene::surface_mesh::RBF_deformer rbf_deformer_left_eye(*left_eye_mesh, src_points_left, tar_points_left);
    // graphene::surface_mesh::RBF_deformer rbf_deformer_right_eye(*right_eye_mesh, src_points_right, tar_points_right);

    // graphene::surface_mesh::RBF_deformer rbf_deformer_left_eye(*left_eye_mesh, src_points_both, tar_points_both);
    // graphene::surface_mesh::RBF_deformer rbf_deformer_right_eye(*right_eye_mesh, src_points_both, tar_points_both);



    // prev view direction-----------------------------------------------------------------------

    //compute eye view directions in undeformed template (vertex 200 is pupil center)
    const Vec3f prev_view_dir_lefteye  = (left_eye_mesh->position(Surface_mesh::Vertex(200)) - undeformed_left_eye_joint_pos).normalize();
    const Vec3f prev_view_dir_righteye = (right_eye_mesh->position(Surface_mesh::Vertex(200)) - undeformed_right_eye_joint_pos).normalize();

    // similarity transformation ----------------------------------------------------------------

    //get transformation in order to transform both eyes according to deformation of template

    //ATTENTION: this performs a minimal depth correction. If you find a more versatile solution for this, implement it!
    const Mat4f M_registration_eyes_left  = Mat4f::translate(Vec3f(0.0f,0.0f,+0.001f)) * graphene::geometry::registration(src_points_left, tar_points_left, graphene::geometry::CONFORMAL_REGISTRATION, NULL, false); // true means no rotation

    const Mat4f M_registration_eyes_right = Mat4f::translate(Vec3f(0.0f,0.0f,+0.001f)) * graphene::geometry::registration(src_points_right, tar_points_right, graphene::geometry::CONFORMAL_REGISTRATION, NULL, false); // true means no rotation

    //transform eye joint positions
    const Vec3f new_lefteye_joint_pos = affine_transform(M_registration_eyes_left, undeformed_left_eye_joint_pos);
    const Vec3f new_righteye_joint_pos = affine_transform(M_registration_eyes_right, undeformed_right_eye_joint_pos);


    // adjust rotation of eye meshes ------------------------------------------------------------

//1 to enable view direction correction
#if 0
    //get new pupil position of both eyes after transform
    const Vec3f left_pupil_pos_transformed = affine_transform(M_registration_eyes_left, left_eye_mesh->position(Surface_mesh::Vertex(200)));
    const Vec3f right_pupil_pos_transformed = affine_transform(M_registration_eyes_right, right_eye_mesh->position(Surface_mesh::Vertex(200)));

    //compute current view direction
    const Vec3f viewing_dir_left_eye  = (left_pupil_pos_transformed  - new_lefteye_joint_pos).normalize();
    const Vec3f viewing_dir_right_eye = (right_pupil_pos_transformed - new_righteye_joint_pos).normalize();

    //const Vec3f viewing_dir_target    = (viewing_dir_left_eye + viewing_dir_right_eye).normalize();

    //compute rotation matrix for left eye
    const double cos_angle_good_left   = dot(viewing_dir_left_eye, prev_view_dir_lefteye);
    const double angle_good_left       = acos( cos_angle_good_left ) * 180.0 / M_PI;
    const Vec3f  axis_good_left        = cross(viewing_dir_left_eye, prev_view_dir_lefteye);
    const Mat4f  M_rotate_good_left    = Mat4f::rotate(axis_good_left, angle_good_left);

    //...for right eye
    const double cos_angle_good_right   = dot(viewing_dir_right_eye, prev_view_dir_righteye);
    const double angle_good_right       = acos( cos_angle_good_right ) * 180.0 / M_PI;
    const Vec3f  axis_good_right        = cross(viewing_dir_right_eye, prev_view_dir_righteye);
    const Mat4f  M_rotate_good_right    = Mat4f::rotate(axis_good_right, angle_good_right);

    //full rotation with translation to origin
    const Mat4f M_rotation_lefteye = Mat4f::translate(new_lefteye_joint_pos) * M_rotate_good_left * Mat4f::translate(-new_lefteye_joint_pos);
    const Mat4f M_rotation_righteye = Mat4f::translate(new_righteye_joint_pos) * M_rotate_good_right * Mat4f::translate(-new_righteye_joint_pos);

    //transform the actual meshes once
    transform_mesh(*left_eye_mesh, M_rotation_lefteye * M_registration_eyes_left);
    transform_mesh(*right_eye_mesh,M_rotation_righteye * M_registration_eyes_right);
#else
    //transform the actual meshes once
    transform_mesh(*left_eye_mesh,  M_registration_eyes_left);
    transform_mesh(*right_eye_mesh, M_registration_eyes_right);
#endif




/*
    for (auto v : left_eye_mesh->vertices())
    {
        left_eye_mesh->position(v) -= bary_left_eye_after;
    }
    transform_mesh(*left_eye_mesh, M_rotate_good_left);
    for (auto v : left_eye_mesh->vertices())
    {
        left_eye_mesh->position(v) += bary_left_eye_after;
    }

    for (auto v : right_eye_mesh->vertices())
    {
        right_eye_mesh->position(v) -= bary_right_eye_after;
    }
    transform_mesh(*right_eye_mesh, M_rotate_good_right);
    for (auto v : right_eye_mesh->vertices())
    {
        right_eye_mesh->position(v) += bary_right_eye_after;
    }

    left_eye_mesh->update_face_normals();
    left_eye_mesh->update_vertex_normals();
    right_eye_mesh->update_face_normals();
    right_eye_mesh->update_vertex_normals();
*/


    // translate eye joints ---------------------------------------------------------------------

    //skeleton.align_axes();

    character::Joint* left_eye_joint  = skeleton.get_joint("LeftEye");
    character::Joint* left_eye_joint_parent = left_eye_joint->parent_;
    character::Joint* right_eye_joint = skeleton.get_joint("RightEye");
    character::Joint* right_eye_joint_parent = right_eye_joint->parent_;

    Vec3f offset_r = new_righteye_joint_pos - right_eye_joint_parent->get_global_translation();
    Vec3f offset_l = new_lefteye_joint_pos  - left_eye_joint_parent->get_global_translation();

    offset_r = (Mat3f(right_eye_joint_parent->global_inv_)) * offset_r;
    offset_l = (Mat3f(left_eye_joint_parent->global_inv_)) * offset_l;

    left_eye_joint->local_ = Mat4f::identity();
    left_eye_joint->local_[12] = offset_l[0];
    left_eye_joint->local_[13] = offset_l[1];
    left_eye_joint->local_[14] = offset_l[2];
    left_eye_joint->bind_pose_local_ = left_eye_joint->local_;

    right_eye_joint->local_ = Mat4f::identity();
    right_eye_joint->local_[12] = offset_r[0];
    right_eye_joint->local_[13] = offset_r[1];
    right_eye_joint->local_[14] = offset_r[2];
    right_eye_joint->bind_pose_local_ = right_eye_joint->local_;

    skeleton.init();

    return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
update_eyes_ver3(const graphene::surface_mesh::Surface_mesh& mesh_left_eye_proxy,
                 const graphene::surface_mesh::Surface_mesh& mesh_right_eye_proxy,
                 graphene::character::Surface_mesh_skin* left_eye_mesh,
                 graphene::character::Surface_mesh_skin* right_eye_mesh,
                 graphene::character::Skeleton& skeleton)
{
    // barycenter of left eye proxy
    Point bary_left_eye_proxy(0.0,0.0,0.0);
    for (auto v : mesh_left_eye_proxy.vertices())
    {
        bary_left_eye_proxy += mesh_left_eye_proxy.position(v);
    }
    bary_left_eye_proxy /= mesh_left_eye_proxy.n_vertices();

    // barycenter of right eye proxy
    Point bary_right_eye_proxy(0.0,0.0,0.0);
    for (auto v : mesh_right_eye_proxy.vertices())
    {
        bary_right_eye_proxy += mesh_right_eye_proxy.position(v);
    }
    bary_right_eye_proxy /= mesh_right_eye_proxy.n_vertices();

    // barycenter of left eye before transformation
    Point bary_left_eye_before(0.0,0.0,0.0);
    for (auto v : left_eye_mesh->vertices())
    {
        bary_left_eye_before += left_eye_mesh->position(v);
    }
    bary_left_eye_before /= left_eye_mesh->n_vertices();

    // barycenter of right eye before transformation
    Point bary_right_eye_before(0.0,0.0,0.0);
    for (auto v : right_eye_mesh->vertices())
    {
        bary_right_eye_before += right_eye_mesh->position(v);
    }
    bary_right_eye_before /= right_eye_mesh->n_vertices();    

    const Vec3f diff_left_eye = bary_left_eye_proxy - bary_left_eye_before;
    const Mat4f initial_trans_left_eye = Mat4f::translate(diff_left_eye);
    transform_mesh(*left_eye_mesh, initial_trans_left_eye);

    const Vec3f diff_right_eye = bary_right_eye_proxy - bary_right_eye_before;
    const Mat4f initial_trans_right_eye = Mat4f::translate(diff_right_eye);
    transform_mesh(*right_eye_mesh, initial_trans_right_eye);

    return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
compute_3d_contour_target_points( const Surface_mesh& mesh_left_eye,
                                  const Surface_mesh& mesh_right_eye,
                                  Surface_mesh& template_mesh,
                                  const Mat4f& rigid_transformation) const
{
    if ( !agi_camera_.is_initialized() )
    {
        std::cerr << "[ERROR] cannot fit eyeball proxfies. Agi_camera is not inizialized!" << std::endl;
        return false;
    }

    if ( (mesh_left_eye.n_vertices()     == 0) ||
         (mesh_right_eye.n_vertices()    == 0) ||
         (template_mesh.n_vertices()     == 0) )
    {
        std::cerr << "[ERROR] can't compute 3d contour target points: invalid input data" << std::endl;
        return false;
    }


    Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh.mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh.mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];
    contour_targets_lefteye.clear();
    contour_targets_righteye.clear();

    Surface_mesh copy_left_eye_mesh  = mesh_left_eye;
    Surface_mesh copy_right_eye_mesh = mesh_right_eye;

    const Mat4f M_inv = inverse(rigid_transformation);

    transform_mesh(copy_left_eye_mesh, M_inv);
    transform_mesh(copy_right_eye_mesh, M_inv);

    const size_t size_div_2 = pixel_coords_eyecontour_.size() / 2;

    // shoot rays and compute intersection with triangles of 'eye_mesh_'
    for (size_t z = 0; z < pixel_coords_eyecontour_.size(); ++z)
    {
        // get center and direction
        Eigen::Vector3d ray_center;
        Eigen::Vector3d ray_direction;
        const double &pixel_coord_x = pixel_coords_eyecontour_[z][0];
        const double &pixel_coord_y = pixel_coords_eyecontour_[z][1];
        agi_camera_.shoot_ray( pixel_coord_x, pixel_coord_y, ray_center, ray_direction );


        // transform eye-meshes back to fit to original camera coordinate system
        Surface_mesh* tmp_trans_eye_mesh;
        if ( z < size_div_2 ) // right eye
        {
            tmp_trans_eye_mesh = &copy_right_eye_mesh;
        }
        else // left eye
        {
            tmp_trans_eye_mesh = &copy_left_eye_mesh;
        }

        auto tmp_trans_eye_mesh_points  = tmp_trans_eye_mesh->vertex_property<Point>("v:point");

        // compute intersection 'ray <-> tmp_trans_eye_mesh'
        bool found_intersection = false;
        Vec3d  p_result;
        double t_result = DBL_MAX;
        Point  p_tmp;
        double t_tmp;
        // for each triangle
        for ( auto tri : tmp_trans_eye_mesh->faces() )
        {
            auto fvit = tmp_trans_eye_mesh->vertices( tri );
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point tri_p0 = tmp_trans_eye_mesh_points[v0];
            const Point tri_p1 = tmp_trans_eye_mesh_points[v1];
            const Point tri_p2 = tmp_trans_eye_mesh_points[v2];

            // does ray intersect triangle?
            if ( intersect_ray_triangle( tri_p0, tri_p1, tri_p2, Point(ray_center[0], ray_center[1], ray_center[2]), Vec3d(ray_direction[0], ray_direction[1], ray_direction[2]), false, p_tmp, t_tmp) )
            {
                // is intersection closer than previous intersections?
                if (t_tmp < t_result)
                {
                    // store data of this intersection
                    t_result   = t_tmp;
                    p_result   = Vec3d(p_tmp);
                }
            }
        }


        if ( t_result != DBL_MAX )
        {
            found_intersection = true;
        }
        if ( !found_intersection )
        {
            std::cerr << "[ERROR] can't compute 3d contour target points: no intersection found" << std::endl;
            return false;
        }

        // transform target to correct coordinate system
        const Vec3f tmp_vec = affine_transform(rigid_transformation, Vec3f( p_result[0], p_result[1], p_result[2] ) );
        p_result = Vec3d( tmp_vec[0], tmp_vec[1], tmp_vec[2] );

        if ( z < size_div_2 ) // right eye
        {
            contour_targets_righteye.push_back( p_result );
        }
        else // left eye
        {
            contour_targets_lefteye.push_back( p_result );
        }
    }

     return true;
}


//-----------------------------------------------------------------------------


bool
Eyes_helper::
has_valid_eye_color(const Color& point_color, float min_brightness, float max_white_deviation, bool filter_skin) const
{

    // check if pixel is bright enough
    const float thr_brightness = min_brightness;

    if ( (point_color[0] < thr_brightness)// ||
         //(color_eye_point[1] < thr_brightness) ||
         //(color_eye_point[2] < thr_brightness)
         )
    {
        return false;
    }

    const float thr = max_white_deviation;
    //const float avg = (color_eye_point[0] + color_eye_point[1] + color_eye_point[2]) / 3.0f;
    const float m_thr = point_color[0]-thr;
    const float p_thr = point_color[0]+thr;
    if (
            !(
            point_color[1] < p_thr && point_color[1] >= m_thr &&
            point_color[2] < p_thr && point_color[2] >= m_thr
            )
            )
    {
        return false;
    }


    if (filter_skin)
    {
        // check if pixel is not skin
        int c_r = 255 * point_color[0];
        int c_g = 255 * point_color[1];
        int c_b = 255 * point_color[2];

        bool   cond1 = (c_r > 95 && c_g > 40 && c_b > 20);
        double delta = std::max(c_r, std::max(c_g, c_b)) - std::min(c_r, std::min(c_g, c_b));
        bool   cond2 = delta > 15;
        bool   cond3 = std::abs(c_r - c_g) > 15 && c_r > c_g && c_r > c_b;
        if ( cond1 && cond2 && cond3) // skin
        {
            return false;
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


void
Eyes_helper::
register_spheres( const std::vector< Vec3f >& src_left,
                  const std::vector< Vec3f >& dst_left,
                  const std::vector< Vec3f >& src_right,
                  const std::vector< Vec3f >& dst_right,
                  graphene::Mat4f& trans_left,
                  graphene::Mat4f& trans_right,
                  double& relative_change ) const
{
    trans_left  = Mat4f::identity();
    trans_right = Mat4f::identity();


    const int n_left  = src_left.size();
    const int n_right = src_right.size();


    double error_before = 0.0;
    for ( unsigned int i = 0; i < n_left; ++i )
    {
        error_before += sqrnorm( dst_left[i] - src_left[i] );
    }
    for ( unsigned int i = 0; i < n_right; ++i )
    {
        error_before += sqrnorm( dst_right[i] - src_right[i] );
    }


    // compute barycenters
    Vector<Scalar,3> scog_left(0.0,0.0,0.0), dcog_left(0.0,0.0,0.0);
    for (int i=0; i<n_left; ++i)
    {
        scog_left += src_left[i];
        dcog_left += dst_left[i];
    }
    scog_left /= (Scalar) n_left;
    dcog_left /= (Scalar) n_left;

    Vector<Scalar,3> scog_right(0.0,0.0,0.0), dcog_right(0.0,0.0,0.0);
    for (int i=0; i<n_right; ++i)
    {
        scog_right += src_right[i];
        dcog_right += dst_right[i];
    }
    scog_right /= (Scalar) n_right;
    dcog_right /= (Scalar) n_right;


    const bool use_rotation = true;
    if ( use_rotation )
    {
        // rotation left eye
        {
            // build matrix
            Mat4d M;
            {
                double  xx(0.0), xy(0.0), xz(0.0), yx(0.0), yy(0.0), yz(0.0), zx(0.0), zy(0.0), zz(0.0);
                Vector<Scalar,3>   sp, dp;

                for (int i=0; i<n_left; ++i)
                {
                    sp = src_left[i]; sp -= scog_left;
                    dp = dst_left[i]; dp -= dcog_left;
                    xx += sp[0] * dp[0];
                    xy += sp[0] * dp[1];
                    xz += sp[0] * dp[2];
                    yx += sp[1] * dp[0];
                    yy += sp[1] * dp[1];
                    yz += sp[1] * dp[2];
                    zx += sp[2] * dp[0];
                    zy += sp[2] * dp[1];
                    zz += sp[2] * dp[2];
                }

                M(0,0) =  xx + yy + zz;
                M(1,1) =  xx - yy - zz;
                M(2,2) = -xx + yy - zz;
                M(3,3) = -xx - yy + zz;
                M(1,0) = M(0,1) = yz - zy;
                M(2,0) = M(0,2) = zx - xz;
                M(2,1) = M(1,2) = xy + yx;
                M(3,0) = M(0,3) = xy - yx;
                M(3,1) = M(1,3) = zx + xz;
                M(3,2) = M(2,3) = yz + zy;
            }


            // symmetric eigendecomposition
            Mat4d   V = Mat4d::identity();
            unsigned int iter(50);
            {
                int     i, j, k;
                double  theta, t, c, s, ss, g, h, tau, tM;

                while (--iter)
                {
                    // find largest off-diagonal element
                    i=0; j=1; ss=fabs(M(0,1));
                    if ( (s=fabs(M(0,2))) > ss) { ss=s; i=0; j=2; }
                    if ( (s=fabs(M(0,3))) > ss) { ss=s; i=0; j=3; }
                    if ( (s=fabs(M(1,2))) > ss) { ss=s; i=1; j=2; }
                    if ( (s=fabs(M(1,3))) > ss) { ss=s; i=1; j=3; }
                    if ( (s=fabs(M(2,3))) > ss) { ss=s; i=2; j=3; }

                    // converged?
                    if (ss < 1e-10) break;

                    // compute Jacobi rotation
                    theta = 0.5 * (M(j,j) - M(i,i)) / M(i,j);
                    t     = (theta<0.0 ? -1.0 : 1.0) / (fabs(theta) + sqrt(1.0+theta*theta));
                    c     = 1.0 / sqrt(1.0 + t*t);
                    s     = t*c;
                    tau   = s/(1.0+c);
                    tM    = t*M(i,j);

#define rot(a, s, t, i, j, k, l)                                        \
                    { g=a(i,j); h=a(k,l); a(i,j)=g-s*(h+g*t); a(k,l)=h+s*(g-h*t); }

                    M(i,j)  = 0.0;
                    for (k=  0; k<i; ++k)  rot(M, s, tau, k, i, k, j);
                    for (k=i+1; k<j; ++k)  rot(M, s, tau, i, k, k, j);
                    for (k=j+1; k<4; ++k)  rot(M, s, tau, i, k, j, k);
                    for (k=  0; k<4; ++k)  rot(V, s, tau, k, i, k, j);
                    M(i,i) -= tM;
                    M(j,j) += tM;
                }
            }


            // did it work?
            if (!iter)
            {
                std::cerr << "registration: Jacobi did not converge\n";
                return;
            }


            // eigenvector wrt largest eigenvalue -> quaternion
            Vec4d q;
            {
                int imax=0;
                double s, ss = M(imax,imax);
                if ( (s=M(1,1)) > ss) { ss=s; imax=1; }
                if ( (s=M(2,2)) > ss) { ss=s; imax=2; }
                if ( (s=M(3,3)) > ss) { ss=s; imax=3; }
                q = Vec4d( V(0,imax), V(1,imax), V(2,imax), V(3,imax) );
                q.normalize();
            }


            // rotation part
            Scalar
                ww(q[0]*q[0]), xx(q[1]*q[1]), yy(q[2]*q[2]), zz(q[3]*q[3]),
                wx(q[0]*q[1]), wy(q[0]*q[2]), wz(q[0]*q[3]),
                xy(q[1]*q[2]), xz(q[1]*q[3]), yz(q[2]*q[3]);
//        Mat4<Scalar> T;
            trans_left(0,0) = ww + xx - yy - zz;
            trans_left(1,0) = 2.0*(xy + wz);
            trans_left(2,0) = 2.0*(xz - wy);
            trans_left(3,0) = 0.0;
            trans_left(0,1) = 2.0*(xy - wz);
            trans_left(1,1) = ww - xx + yy - zz;
            trans_left(2,1) = 2.0*(yz + wx);
            trans_left(3,1) = 0.0;
            trans_left(0,2) = 2.0*(xz + wy);
            trans_left(1,2) = 2.0*(yz - wx);
            trans_left(2,2) = ww - xx - yy + zz;
            trans_left(3,2) = 0.0;
        }


        // rotation right eye
        {
            // build matrix
            Mat4d M;
            {
                double  xx(0.0), xy(0.0), xz(0.0), yx(0.0), yy(0.0), yz(0.0), zx(0.0), zy(0.0), zz(0.0);
                Vector<Scalar,3>   sp, dp;

                for (int i=0; i<n_right; ++i)
                {
                    sp = src_right[i]; sp -= scog_right;
                    dp = dst_right[i]; dp -= dcog_right;
                    xx += sp[0] * dp[0];
                    xy += sp[0] * dp[1];
                    xz += sp[0] * dp[2];
                    yx += sp[1] * dp[0];
                    yy += sp[1] * dp[1];
                    yz += sp[1] * dp[2];
                    zx += sp[2] * dp[0];
                    zy += sp[2] * dp[1];
                    zz += sp[2] * dp[2];
                }

                M(0,0) =  xx + yy + zz;
                M(1,1) =  xx - yy - zz;
                M(2,2) = -xx + yy - zz;
                M(3,3) = -xx - yy + zz;
                M(1,0) = M(0,1) = yz - zy;
                M(2,0) = M(0,2) = zx - xz;
                M(2,1) = M(1,2) = xy + yx;
                M(3,0) = M(0,3) = xy - yx;
                M(3,1) = M(1,3) = zx + xz;
                M(3,2) = M(2,3) = yz + zy;
            }


            // symmetric eigendecomposition
            Mat4d   V = Mat4d::identity();
            unsigned int iter(50);
            {
                int     i, j, k;
                double  theta, t, c, s, ss, g, h, tau, tM;

                while (--iter)
                {
                    // find largest off-diagonal element
                    i=0; j=1; ss=fabs(M(0,1));
                    if ( (s=fabs(M(0,2))) > ss) { ss=s; i=0; j=2; }
                    if ( (s=fabs(M(0,3))) > ss) { ss=s; i=0; j=3; }
                    if ( (s=fabs(M(1,2))) > ss) { ss=s; i=1; j=2; }
                    if ( (s=fabs(M(1,3))) > ss) { ss=s; i=1; j=3; }
                    if ( (s=fabs(M(2,3))) > ss) { ss=s; i=2; j=3; }

                    // converged?
                    if (ss < 1e-10) break;

                    // compute Jacobi rotation
                    theta = 0.5 * (M(j,j) - M(i,i)) / M(i,j);
                    t     = (theta<0.0 ? -1.0 : 1.0) / (fabs(theta) + sqrt(1.0+theta*theta));
                    c     = 1.0 / sqrt(1.0 + t*t);
                    s     = t*c;
                    tau   = s/(1.0+c);
                    tM    = t*M(i,j);

#define rot(a, s, t, i, j, k, l)                                        \
                    { g=a(i,j); h=a(k,l); a(i,j)=g-s*(h+g*t); a(k,l)=h+s*(g-h*t); }

                    M(i,j)  = 0.0;
                    for (k=  0; k<i; ++k)  rot(M, s, tau, k, i, k, j);
                    for (k=i+1; k<j; ++k)  rot(M, s, tau, i, k, k, j);
                    for (k=j+1; k<4; ++k)  rot(M, s, tau, i, k, j, k);
                    for (k=  0; k<4; ++k)  rot(V, s, tau, k, i, k, j);
                    M(i,i) -= tM;
                    M(j,j) += tM;
                }
            }


            // did it work?
            if (!iter)
            {
                std::cerr << "registration: Jacobi did not converge\n";
                return;
            }


            // eigenvector wrt largest eigenvalue -> quaternion
            Vec4d q;
            {
                int imax=0;
                double s, ss = M(imax,imax);
                if ( (s=M(1,1)) > ss) { ss=s; imax=1; }
                if ( (s=M(2,2)) > ss) { ss=s; imax=2; }
                if ( (s=M(3,3)) > ss) { ss=s; imax=3; }
                q = Vec4d( V(0,imax), V(1,imax), V(2,imax), V(3,imax) );
                q.normalize();
            }


            // rotation part
            Scalar
                ww(q[0]*q[0]), xx(q[1]*q[1]), yy(q[2]*q[2]), zz(q[3]*q[3]),
                wx(q[0]*q[1]), wy(q[0]*q[2]), wz(q[0]*q[3]),
                xy(q[1]*q[2]), xz(q[1]*q[3]), yz(q[2]*q[3]);
//        Mat4<Scalar> T;
            trans_right(0,0) = ww + xx - yy - zz;
            trans_right(1,0) = 2.0*(xy + wz);
            trans_right(2,0) = 2.0*(xz - wy);
            trans_right(3,0) = 0.0;
            trans_right(0,1) = 2.0*(xy - wz);
            trans_right(1,1) = ww - xx + yy - zz;
            trans_right(2,1) = 2.0*(yz + wx);
            trans_right(3,1) = 0.0;
            trans_right(0,2) = 2.0*(xz + wy);
            trans_right(1,2) = 2.0*(yz - wx);
            trans_right(2,2) = ww - xx - yy + zz;
            trans_right(3,2) = 0.0;
        }
    }



    if (true)
    {
        const bool scale_separate = false;
        if ( scale_separate )
        {
            // scaling
            Scalar  nom_left(0), denom_left(0);
            Vector<Scalar,3>  sp_left, dp_left;
            for (int i=0; i<n_left; ++i)
            {
                sp_left = src_left[i]; sp_left -= scog_left;
                dp_left = dst_left[i]; dp_left -= dcog_left;

                sp_left = linear_transform(trans_left, sp_left);

                nom_left   += dot(sp_left,dp_left);
                denom_left += dot(sp_left,sp_left);
            }
            const Scalar scaling_left = nom_left / denom_left;
            Scalar  nom_right(0), denom_right(0);
            Vector<Scalar,3>  sp_right, dp_right;
            for (int i=0; i<n_right; ++i)
            {
                sp_right = src_right[i]; sp_right -= scog_right;
                dp_right = dst_right[i]; dp_right -= dcog_right;

                sp_right = linear_transform(trans_right, sp_right);

                nom_right   += dot(sp_right,dp_right);
                denom_right += dot(sp_right,sp_right);
            }
            const Scalar scaling_right = nom_right / denom_right;


            for (int i=0; i<3; ++i)
            {
                for (int j=0; j<3; ++j)
                {
                    trans_left(i,j)  *= scaling_left;
                    trans_right(i,j) *= scaling_right;
                }
            }
        }
        else
        {
            // scaling
            Scalar  nom(0), denom(0);
            Vector<Scalar,3>  sp_left, dp_left;
            for (int i=0; i<n_left; ++i)
            {
                sp_left = src_left[i]; sp_left -= scog_left;
                dp_left = dst_left[i]; dp_left -= dcog_left;

                sp_left = linear_transform(trans_left, sp_left);

                nom   += dot(sp_left,dp_left);
                denom += dot(sp_left,sp_left);
            }
            Vector<Scalar,3>  sp_right, dp_right;
            for (int i=0; i<n_right; ++i)
            {
                sp_right = src_right[i]; sp_right -= scog_right;
                dp_right = dst_right[i]; dp_right -= dcog_right;

                sp_right = linear_transform(trans_right, sp_right);

                nom   += dot(sp_right,dp_right);
                denom += dot(sp_right,sp_right);
            }
            const Scalar scaling_total = nom / denom;

            for (int i=0; i<3; ++i)
            {
                for (int j=0; j<3; ++j)
                {
                    trans_left(i,j)  *= scaling_total;
                    trans_right(i,j) *= scaling_total;
                }
            }
        }
    }


    // translation part
    trans_left(0,3) = dcog_left[0] - trans_left(0,0)*scog_left[0] - trans_left(0,1)*scog_left[1] - trans_left(0,2)*scog_left[2];
    trans_left(1,3) = dcog_left[1] - trans_left(1,0)*scog_left[0] - trans_left(1,1)*scog_left[1] - trans_left(1,2)*scog_left[2];
    trans_left(2,3) = dcog_left[2] - trans_left(2,0)*scog_left[0] - trans_left(2,1)*scog_left[1] - trans_left(2,2)*scog_left[2];
    trans_left(3,3) = 1.0;

    trans_right(0,3) = dcog_right[0] - trans_right(0,0)*scog_right[0] - trans_right(0,1)*scog_right[1] - trans_right(0,2)*scog_right[2];
    trans_right(1,3) = dcog_right[1] - trans_right(1,0)*scog_right[0] - trans_right(1,1)*scog_right[1] - trans_right(1,2)*scog_right[2];
    trans_right(2,3) = dcog_right[2] - trans_right(2,0)*scog_right[0] - trans_right(2,1)*scog_right[1] - trans_right(2,2)*scog_right[2];
    trans_right(3,3) = 1.0;


    double error_after = 0.0;
    for ( unsigned int i = 0; i < n_left; ++i )
    {
        Vec3f sp_left = src_left[i];
        sp_left = affine_transform(trans_left, sp_left);

        error_after += sqrnorm( dst_left[i] - sp_left );
    }
    for ( unsigned int i = 0; i < n_right; ++i )
    {
        Vec3f sp_right = src_right[i];
        sp_right = affine_transform(trans_right, sp_right);

        error_after += sqrnorm( dst_right[i] - sp_right );
    }


    if (error_after > error_before)
    {
        std::cerr << "ERROR in register_spheres(...)" << std::endl;
        return;
    }
    relative_change = (error_before - error_after) / error_before;
}


void Eyes_helper::rgb_to_hsl(const Vec3f &rgb, Vec3f &hsl) const
{
    int c_max = 0;
    float v_max = -FLT_MAX;
    for (int i=0; i < 3; ++i)
    {
        if (rgb[i] > v_max)
        {
            v_max = rgb[i];
            c_max = i;
        }
    }

    int c_min = 0;
    float v_min = FLT_MAX;
    for (int i=0; i < 3; ++i)
    {
        if (rgb[i] < v_min)
        {
            v_min = rgb[i];
            c_min = i;
        }
    }

    float &l = hsl[2];
    l = (v_max + v_min) * 0.5f;

    if (l < 0.5)
    {
        hsl[1] = (v_max - v_min) / (v_max+v_min);
    }
    else
    {
        hsl[1] = (v_max - v_min) / (2-(v_max+v_min));
    }

    switch (c_max)
    {
    case 0:
    {
        hsl[0] = 60.0f * ((rgb[1] - rgb[2]) / (v_max - v_min));
    }
        break;
    case 1:
    {
        hsl[0] = 120.0f + 60.0f * ((rgb[2] - rgb[0]) / (v_max - v_min));
    }
        break;
    case 2:
    {
        hsl[0] = 240.0f + 60.0f * ((rgb[0] - rgb[1]) / (v_max - v_min));
    }
        break;
    default:
        break;
    }


    return;
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
