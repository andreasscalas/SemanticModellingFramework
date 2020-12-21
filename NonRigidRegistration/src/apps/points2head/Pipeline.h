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

    scene_graph::Surface_mesh_node* template_model_;
    scene_graph::Point_set_node*    pointset_head_;

public:
    Pipeline();

    ~Pipeline();

    void run();

private:
    //input/output
    bool load();
    bool save();

    //head
    ///select ear landmarks on head pointset
    bool head_ear_landmarks_selection();
    ///detect or select facial features and eventually map them to face pointset
    bool head_landmarks_facial_features();
    ///start head fitting
    bool head_fitting();
    ///initial alignment of head pointset and template
    bool head_initial_alignment();
    ///non-rigid fit with closest point correspondences
    bool head_cpc_fitting();
};

}

#endif
