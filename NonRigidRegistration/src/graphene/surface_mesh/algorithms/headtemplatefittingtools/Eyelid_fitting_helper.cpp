
#include "Eyelid_fitting_helper.h"
#include "Eyes_helper.h"


namespace graphene {
namespace eyelid_fitting_helper {

using namespace surface_mesh;

bool compute_eye_contour_target_points(const Eyelid_info &eyelid_info, const geometry::Point_set& original_pointset, character::Character &ch)
{
    surface_mesh::Eyes_helper eyes_helper;

    //load selected contour pixel coordinates
    if (! eyes_helper.load_contour_pixelcoords(eyelid_info.fn_contour_eyes))
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Unable to load eye contour pixel coordinates from file: \"" << eyelid_info.fn_contour_eyes << "\"." << std::endl;
        return false;
    }

    //load agi camera
    if (! eyes_helper.load_frontal_camera(eyelid_info.fn_cameras, eyelid_info.fn_frontal_image))
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Unable to load camera from file: \"" << eyelid_info.fn_cameras << "\"." << std::endl;
        return false;
    }


    const std::vector<character::Surface_mesh_skin*> &skins = ch.skins();
    surface_mesh::Surface_mesh eyetrans_left,eyetrans_right;
    surface_mesh::Surface_mesh *body_mesh;

    if (eyetrans_left.read(eyelid_info.fn_eye_proxy))
    {
	eyetrans_left.update_face_normals();
	eyetrans_left.update_vertex_normals();
        eyetrans_right = eyetrans_left;
    }
    else
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [WARNING] Unable to load eye proxy from file: \"" << eyelid_info.fn_eye_proxy
                  << "\".\nTrying to use eye mesh from template." << std::endl;
        for (size_t i = 0; i < skins.size(); ++i)
        {
            auto skin_id = skins[i]->mesh_property<std::string>("m:id");

            //COPY
            if ( skin_id[0] == eyelid_info.s_eyetrans_left )
            {
                eyetrans_left = *skins[i];
            }
            //COPY
            else if ( skin_id[0] == eyelid_info.s_eyetrans_right )
            {
                eyetrans_right = *skins[i];
            }
        }
    }


    for (size_t i = 0; i < skins.size(); ++i)
    {
        auto skin_id = skins[i]->mesh_property<std::string>("m:id");

        //POINTER
        if ( skin_id[0] == eyelid_info.s_bodymesh)
        {
            body_mesh = skins[i];
        }
    }

    if (eyetrans_left.empty() || eyetrans_right.empty() || body_mesh == nullptr)
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points [ERROR] Unable to find eye proxies or base mesh." << std::endl;
        return false;
    }


    //load template contour selections
    //right eye
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_righteye
        = body_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_righteye");
    std::vector<Surface_mesh::Vertex>& contour_points_right_eye = contour_righteye[0];
    //left eye
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_lefteye
        = body_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_lefteye");
    std::vector<Surface_mesh::Vertex>& contour_points_left_eye = contour_lefteye[0];

    //load both selections
    if (
            ! load_selection_from_file(contour_points_left_eye, eyelid_info.fn_sel_eyecontour_left)
            ||
            ! load_selection_from_file(contour_points_right_eye, eyelid_info.fn_sel_eyecontour_right)
            )
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Unable to load template eye contour selection." << std::endl;
        return false;
    }

    if (eyes_helper.pixel_coords_eyecontour_.size() != (contour_points_left_eye.size() + contour_points_right_eye.size()) )
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Wrong number of contour points.\n"
                  << "    - Either wrong image contour points or wrong template contour points loaded."
                  << std::endl;
        return false;
    }

    std::cout << "eyelid_fitting_helper::compute_eye_contour_target_points: [STATUS] Fitting eyeball proxies." << std::endl;

    //fit eyeball proxies
    if (! eyes_helper.fit_eyeball_proxies(eyelid_info.fn_frontal_image, eyetrans_left, eyetrans_right, original_pointset, *body_mesh, eyelid_info.pointset_to_mesh_transformation))
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Unable to fit eye proxies." << std::endl;
        return false;
    }

    std::cout << "eyelid_fitting_helper::compute_eye_contour_target_points: [STATUS] Computing contour target points." << std::endl;

    //compute contour target points
    if (! eyes_helper.compute_3d_contour_target_points(eyetrans_left, eyetrans_right, *body_mesh, eyelid_info.pointset_to_mesh_transformation))
    {
        std::cerr << "eyelid_fitting_helper::compute_eye_contour_target_points: [ERROR] Unable to compute contour target points." << std::endl;
        return false;
    }


    return true;
}

}
}
