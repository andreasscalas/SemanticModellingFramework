//=============================================================================
#ifndef GRAPHENE_EYELID_FITTING_HELPER_H
#define GRAPHENE_EYELID_FITTING_HELPER_H
//=============================================================================

//== INCLUDES =================================================================

#include <graphene/character/data_structure/Character.h>
#include <graphene/geometry/Point_set.h>

//== NAMESPACE ================================================================

namespace graphene {
namespace eyelid_fitting_helper {

struct Eyelid_info
{
    std::string s_eyetrans_left;
    std::string s_eyetrans_right;
    std::string s_bodymesh;
    std::string fn_cameras;
    std::string fn_frontal_image;
    std::string fn_contour_eyes;
    std::string fn_sel_eyecontour_left;
    std::string fn_sel_eyecontour_right;
    std::string fn_eye_proxy;

    Mat4f pointset_to_mesh_transformation;
};

bool compute_eye_contour_target_points(const Eyelid_info &eyelid_info, const geometry::Point_set &original_pointset, character::Character& ch);

//=============================================================================
} // namespace eyelid_fitting_helper
} // namespace graphene
//=============================================================================
#endif
//=============================================================================


