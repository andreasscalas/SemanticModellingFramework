#ifndef PIPELINE_H
#define PIPELINE_H

#include "P2C_window.h"

#include <io/Loader.h>

namespace p2c
{

using namespace surface_mesh;

class Pipeline
{
private:
    P2C_window window_;

    io::Loader loader_;

    scene_graph::Character_node* template_model_;
    scene_graph::Point_set_node* pointset_body_;
    scene_graph::Point_set_node* pointset_face_;
    scene_graph::Point_set_node* pointset_left_hand_;
    scene_graph::Point_set_node* pointset_right_hand_;

public:
    Pipeline();

    ~Pipeline();


    void run();

private:
    //input/output
    bool load();
    bool save();

    //preparation
    /// save undeformed data to transfer blendshapes as well as eye joints
    void save_undeformed_data();

    //body
    ///select landmarks on body pointset
    bool body_landmarks_selection();
    ///start body fitting: posture and shape
    bool body_fitting();
    ///fits posture (including pca fitting)
    bool body_posture_fitting();
    ///fits shape with landmarks only
    void body_landmarkonly_fitting();
    ///non-rigid fit with closest point correspondences
    bool body_cpc_fitting();
    ///compute texture and merge
    bool body_texturize();
    ///finalize: inverse posture and put feet on ground
    bool body_finalize();

    //face
    ///select landmarks on face pointset
    bool face_landmarks_selection();
    ///start face fitting: initial alignment and non-rigid
    bool face_fitting();
    ///detect facial features and maps them to face pointset
    bool face_detect_facial_features();
    ///initial alignment of face pointset and template using facial features
    bool face_initial_alignment();
    ///non-rigid fit with closest point correspondences
    bool face_cpc_fitting();
    ///compute and merge face texture
    bool face_texturize();

    //left hand
    ///select landmarks on left hand pointset
    bool left_hand_landmarks_selection();
    ///start left hand fitting: posture and shape
    bool left_hand_fitting();
    ///fits left hand posture
    bool left_hand_posture_fitting();
    ///fits shape with landmarks only
    void left_hand_landmarkonly_fitting();
    ///non-rigid fit with closest point correspondences
    bool left_hand_cpc_fitting();
    ///compute left hand texture and merge
    bool left_hand_texturize();
    ///finalize: inverse posture
    bool left_hand_finalize();

    //right hand
    ///select landmarks on right hand pointset
    bool right_hand_landmarks_selection();
    ///start right hand fitting: posture and shape
    bool right_hand_fitting();
    ///fits right hand posture
    bool right_hand_posture_fitting();
    ///fits shape with landmarks only
    void right_hand_landmarkonly_fitting();
    ///non-rigid fit with closest point correspondences
    bool right_hand_cpc_fitting();
    ///compute right hand texture and merge
    bool right_hand_texturize();
    ///finalize: inverse posture
    bool right_hand_finalize();

    //both
    ///correct eyes and teeth using similarity transformation
    bool correct_eyes_and_teeth();
    ///transfer blendshapes to fitted model (deformation transfer)
    bool transfer_blendshapes();
};



}


#endif
