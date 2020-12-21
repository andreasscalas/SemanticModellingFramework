//== INCLUDES ===================================================================


#include "Eyes_correction_helper.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>
#include <graphene/geometry/registration.h>
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>


namespace graphene {
namespace eyes_correction_helper {

using namespace surface_mesh;

bool correct_eyes(character::Character &inout_character, const Eye_info &eye_info)
{
    // prepare character ------------------------------------------------------


    graphene::character::Skeleton& skeleton = inout_character.skeleton();

    // prepare skins ------------------------------------------------------

    const std::vector<graphene::character::Surface_mesh_skin*>& skins = inout_character.skins();

    graphene::character::Surface_mesh_skin
            *base_mesh=0,
            *eyemesh_left=0,
            *eyemesh_right=0,
            *eyetrans_left=0,
            *eyetrans_right=0,
            *eyegland_left=0,
            *eyegland_right=0
            ;

    for (size_t i = 0; i < skins.size(); ++i)
    {
        auto skin_id = skins[i]->mesh_property<std::string>("m:id");

        if ( skin_id[0] == eye_info.s_bodymesh )
        {
            base_mesh = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyemesh_left )
        {
            eyemesh_left = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyemesh_right )
        {
            eyemesh_right = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyetrans_left )
        {
            eyetrans_left = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyetrans_right )
        {
            eyetrans_right = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyegland_left )
        {
            eyegland_left = skins[i];
        }
        else if ( skin_id[0] == eye_info.s_eyegland_right )
        {
            eyegland_right = skins[i];
        }
    }

    if (!base_mesh)
    {
        std::cerr << "correct_eyes: [ERROR] Cannot correct eyes. No base mesh found!" << std::endl;
        return false;
    }



    // correct eyes --------------------------------------------------------

    std::vector<unsigned int> selection_left_vec;
    if (!graphene::surface_mesh::load_selection_from_file(selection_left_vec, eye_info.fn_sel_eyeregion_left))
    {
        std::cerr << "correct_eyes: [ERROR] Can't open selection (left eye). Filename: \"" << eye_info.fn_sel_eyeregion_left << "\"." << std::endl;
        return false;
    }
    std::vector<unsigned int> selection_right_vec;
    if (!graphene::surface_mesh::load_selection_from_file(selection_right_vec, eye_info.fn_sel_eyeregion_right))
    {
        std::cerr << "correct_eyes: [ERROR] Can't open selection (right eye). Filename: \"" << eye_info.fn_sel_eyeregion_right << "\"." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<Point> undef_points = base_mesh->get_vertex_property<Point>("v:undef_point");
    if (!undef_points)
    {
        std::cerr << "correct_teeth: [ERROR] No undeformed mesh vertices found." << std::endl;
        return false;
    }

    std::vector<Point> src_points_left;
    std::vector<Point> tar_points_left;
    std::vector<Point> src_points_right;
    std::vector<Point> tar_points_right;
    for ( unsigned int i = 0; i < selection_left_vec.size(); ++i)
    {
        Surface_mesh::Vertex v(selection_left_vec[i]);

        src_points_left.push_back(undef_points[v]);
        tar_points_left.push_back(base_mesh->position(v));
    }
    for ( unsigned int i = 0; i < selection_right_vec.size(); ++i)
    {
        Surface_mesh::Vertex v(selection_right_vec[i]);

        src_points_right.push_back(undef_points[v]);
        tar_points_right.push_back(base_mesh->position(v));

        // src_points_both.push_back(mesh_source_undeformed.position(v));
        // tar_points_both.push_back(mesh_target_undeformed.position(v));
    }


    // similarity transformation ----------------------------------------------------------------

    //get transformation in order to transform both eyes according to deformation of template

    //ATTENTION: this performs a minimal depth correction. If you find a more versatile solution for this, implement it!
    const Mat4f M_registration_eyes_left  = Mat4f::translate(Vec3f(0.0f,0.0f,-0.0005f)) * graphene::geometry::registration(src_points_left, tar_points_left, graphene::geometry::CONFORMAL_REGISTRATION, NULL, false); // true means no rotation

    const Mat4f M_registration_eyes_right = Mat4f::translate(Vec3f(0.0f,0.0f,-0.0005f)) * graphene::geometry::registration(src_points_right, tar_points_right, graphene::geometry::CONFORMAL_REGISTRATION, NULL, false); // true means no rotation


    //transform meshes

    if (eyemesh_left != nullptr && eyemesh_right != nullptr)
    {
        transform_mesh(*eyemesh_left , M_registration_eyes_left);
        transform_mesh(*eyemesh_right, M_registration_eyes_right);
    }
    else
    {
        std::cerr << "correct_eyes: [ERROR] Could not correct eye balls. Eye ball meshes not found!" << std::endl;
    }

    if (eyegland_left != nullptr && eyegland_right != nullptr)
    {
        transform_mesh(*eyegland_left , M_registration_eyes_left);
        transform_mesh(*eyegland_right, M_registration_eyes_right);
    }
    else
    {
        std::cerr << "correct_eyes: [ERROR] Could not correct eye glands. Gland meshes not found!" << std::endl;
    }


    if (eyetrans_left != nullptr && eyetrans_right != nullptr)
    {
        transform_mesh(*eyetrans_left , M_registration_eyes_left);
        transform_mesh(*eyetrans_right, M_registration_eyes_right);
    }
    else
    {
        std::cerr << "correct_eyes: [ERROR] Could not correct transparent eye sphere. Eye sphere meshes not found!" << std::endl;
    }

    // translate eye joints ---------------------------------------------------------------------
    //transform eye joint positions

    Surface_mesh::Mesh_property<Vec3f> undef_leye_pos = base_mesh->get_mesh_property<Vec3f>("m:undef_leye_pos");
    Surface_mesh::Mesh_property<Vec3f> undef_reye_pos = base_mesh->get_mesh_property<Vec3f>("m:undef_reye_pos");

    if (!undef_leye_pos || !undef_reye_pos)
    {
        std::cerr << "correct_eyes: [ERROR] Cannot find undeformed eye joint positions." << std::endl;
        return false;
    }

    const Vec3f undeformed_left_eye_joint_pos = undef_leye_pos[0];
    const Vec3f undeformed_right_eye_joint_pos = undef_reye_pos[0];

    const Vec3f new_lefteye_joint_pos = affine_transform(M_registration_eyes_left, undeformed_left_eye_joint_pos);
    const Vec3f new_righteye_joint_pos = affine_transform(M_registration_eyes_right, undeformed_right_eye_joint_pos);

    character::Joint* left_eye_joint  = skeleton.get_joint(eye_info.s_eyejoint_left.c_str());
    character::Joint* right_eye_joint = skeleton.get_joint(eye_info.s_eyejoint_right.c_str());
    if (!left_eye_joint || !right_eye_joint)
    {
        std::cerr << "correct_eyes: [ERROR] Cannot find eye joints." << std::endl;
        return false;
    }

    character::Joint* left_eye_joint_parent = left_eye_joint->parent_;
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

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

