#ifndef TEMPLATE_CONFIG_H
#define TEMPLATE_CONFIG_H

#include "Base_config.h"

namespace p2c
{
struct Template_config : public Base_config
{
    std::string dir_template_db;
    std::string fn_template;
    std::string fn_landmarks_body;
    std::string fn_landmarks_face_ears;
    std::string fn_landmarks_facialfeatures;
    std::string fn_landmarks_hand_left;
    std::string fn_landmarks_hand_right;
    std::string fn_template_ik_dof;
    std::string fn_template_ik_dof_hand_left;
    std::string fn_template_ik_dof_hand_right;
    std::string fn_pca_body_mean_mesh;
    std::string fn_pca_body;
    std::string fn_vw_fullbody_hands_fitting;
    std::string fn_vw_fullbody_face_fitting;
    std::string fn_vw_face_and_ears_fitting;
    std::string fn_vw_forearms_fitting;
    std::string fn_sel_keep_vertices_eyes;
    std::string fn_sel_keep_body_vertices;
    std::string fn_sel_keep_nohands_vertices;
    std::string fn_sel_mouth_shut;
    std::string fn_sel_sole_vertices;
    std::string fn_sel_sole_vertices_inverse;
    std::string fn_sel_mouth_region;
    std::string fn_sel_eyeregion_left;
    std::string fn_sel_eyeregion_right;
    std::string fn_sel_deftrans_fixed_vertices;
    std::string fn_sel_eye_vertices_to_omit;
    std::string fn_sel_eyecontour_left;
    std::string fn_sel_eyecontour_right;
    std::string fn_mask_eyes_and_teeth_inverse;
    std::string fn_mask_hands_comparison;
    std::string fn_mask_hands_repair;
    std::string fn_mask_left_fingertips_repair;
    std::string fn_mask_right_fingertips_repair;
    std::string fn_mask_hand_left_repair;
    std::string fn_mask_hand_right_repair;    
    std::string fn_mask_hands_dirichlet;
    std::string fn_mask_armpit_repair;
    std::string fn_mask_face_and_ears;
    std::string fn_facialfeatures_model;
    std::string fn_eye_proxy;
    std::string fn_skeleton_convert_info;
    std::string fn_retargeting_skeleton;
    std::string fn_retargeting_excluded_joints;


    std::string s_eyejoint_left;
    std::string s_eyejoint_right;
    std::string s_eyemesh_left;
    std::string s_eyemesh_right;
    std::string s_eyetrans_left;
    std::string s_eyetrans_right;
    std::string s_eyegland_left;
    std::string s_eyegland_right;
    std::string s_bodymesh;
    std::string s_teeth_up;
    std::string s_teeth_down;

    Template_config() {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {

        //find_key(data, "dir_template_db", dir_template_db);


        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-t" && argv.size() > i+1)
            {
                dir_template_db = argv[i+1];
                if (dir_template_db.back() != '/')
                    dir_template_db.push_back('/');
            }
        }

        find_key(data, "fn_template", fn_template, dir_template_db);
        find_key(data, "fn_landmarks_body", fn_landmarks_body, dir_template_db);
        find_key(data, "fn_landmarks_face_ears", fn_landmarks_face_ears, dir_template_db);
        find_key(data, "fn_landmarks_facialfeatures", fn_landmarks_facialfeatures, dir_template_db);
        find_key(data, "fn_landmarks_hand_left", fn_landmarks_hand_left, dir_template_db);
        find_key(data, "fn_landmarks_hand_right", fn_landmarks_hand_right, dir_template_db);
        find_key(data, "fn_template_ik_dof", fn_template_ik_dof, dir_template_db);
        find_key(data, "fn_template_ik_dof_hand_left", fn_template_ik_dof_hand_left, dir_template_db);
        find_key(data, "fn_template_ik_dof_hand_right", fn_template_ik_dof_hand_right, dir_template_db);
        find_key(data, "fn_pca_body_mean_mesh", fn_pca_body_mean_mesh, dir_template_db);
        find_key(data, "fn_pca_body", fn_pca_body, dir_template_db);
        find_key(data, "fn_vw_fullbody_hands_fitting", fn_vw_fullbody_hands_fitting, dir_template_db);
        find_key(data, "fn_vw_fullbody_face_fitting", fn_vw_fullbody_face_fitting, dir_template_db);
        find_key(data, "fn_vw_face_and_ears_fitting", fn_vw_face_and_ears_fitting, dir_template_db);
        find_key(data, "fn_vw_forearms_fitting", fn_vw_forearms_fitting, dir_template_db);
        find_key(data, "fn_sel_keep_vertices_eyes", fn_sel_keep_vertices_eyes, dir_template_db);
        find_key(data, "fn_sel_keep_body_vertices", fn_sel_keep_body_vertices, dir_template_db);
        find_key(data, "fn_sel_keep_nohands_vertices", fn_sel_keep_nohands_vertices, dir_template_db);        
        find_key(data, "fn_sel_mouth_shut", fn_sel_mouth_shut, dir_template_db);
        find_key(data, "fn_sel_sole_vertices", fn_sel_sole_vertices, dir_template_db);
        find_key(data, "fn_sel_sole_vertices_inverse", fn_sel_sole_vertices_inverse, dir_template_db);
        find_key(data, "fn_sel_mouth_region", fn_sel_mouth_region, dir_template_db);
        find_key(data, "fn_sel_eyeregion_left", fn_sel_eyeregion_left, dir_template_db);
        find_key(data, "fn_sel_eyeregion_right", fn_sel_eyeregion_right, dir_template_db);
        find_key(data, "fn_sel_deftrans_fixed_vertices", fn_sel_deftrans_fixed_vertices, dir_template_db);
        find_key(data, "fn_sel_eye_vertices_to_omit", fn_sel_eye_vertices_to_omit, dir_template_db);
        find_key(data, "fn_sel_eyecontour_left", fn_sel_eyecontour_left, dir_template_db);
        find_key(data, "fn_sel_eyecontour_right", fn_sel_eyecontour_right, dir_template_db);
        find_key(data, "fn_mask_eyes_and_teeth_inverse", fn_mask_eyes_and_teeth_inverse, dir_template_db);
        find_key(data, "fn_mask_hands_comparison", fn_mask_hands_comparison, dir_template_db);
        find_key(data, "fn_mask_hands_repair", fn_mask_hands_repair, dir_template_db);
        find_key(data, "fn_mask_left_fingertips_repair", fn_mask_left_fingertips_repair, dir_template_db);
        find_key(data, "fn_mask_right_fingertips_repair", fn_mask_right_fingertips_repair, dir_template_db);
        find_key(data, "fn_mask_hand_left_repair", fn_mask_hand_left_repair, dir_template_db);
        find_key(data, "fn_mask_hand_right_repair", fn_mask_hand_right_repair, dir_template_db);        
        find_key(data, "fn_mask_hands_dirichlet", fn_mask_hands_dirichlet, dir_template_db);
        find_key(data, "fn_mask_armpit_repair", fn_mask_armpit_repair, dir_template_db);
        find_key(data, "fn_mask_face_and_ears", fn_mask_face_and_ears, dir_template_db);
        find_key(data, "fn_facialfeatures_model", fn_facialfeatures_model, dir_template_db);
        find_key(data, "fn_eye_proxy", fn_eye_proxy, dir_template_db);
        find_key(data, "fn_skeleton_convert_info", fn_skeleton_convert_info, dir_template_db);
        find_key(data, "fn_retargeting_skeleton", fn_retargeting_skeleton, dir_template_db);
        find_key(data, "fn_retargeting_excluded_joints", fn_retargeting_excluded_joints, dir_template_db);


        find_key(data,"s_eyejoint_left", s_eyejoint_left);
        find_key(data,"s_eyejoint_right", s_eyejoint_right);
        find_key(data,"s_eyemesh_left", s_eyemesh_left);
        find_key(data,"s_eyemesh_right", s_eyemesh_right);
        find_key(data,"s_eyetrans_left", s_eyetrans_left);
        find_key(data,"s_eyetrans_right", s_eyetrans_right);
        find_key(data,"s_eyegland_left", s_eyegland_left);
        find_key(data,"s_eyegland_right", s_eyegland_right);
        find_key(data,"s_bodymesh", s_bodymesh);
        find_key(data,"s_teeth_up", s_teeth_up);
        find_key(data,"s_teeth_down", s_teeth_down);
    }
};
}


#endif
