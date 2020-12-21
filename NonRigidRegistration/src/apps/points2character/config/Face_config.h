#ifndef FACE_CONFIG_H
#define FACE_CONFIG_H

#include "Base_config.h"

namespace p2c
{

struct Face_config : Base_config
{
    std::string dir_project;
    std::string fn_pointset;
    std::string fn_invtrans_mesh;
    std::string fn_photoscan_project;
    std::string fn_texture;
    std::string fn_frontal_image;
    std::string fn_contour_eyes;
    std::string fn_ears_landmarks;
    std::string fn_cameras;
    std::string fn_landmarks_facialfeatures;

    int int_detail_level;

    double dbl_mouth_shut_weight;
    double dbl_eyelid_weight;
    double dbl_ears_lm_weight;
    double dbl_keep_body_vertices_weight;

    bool b_textured;
    bool b_provide_texture_manually;
    bool b_manual_eye_contour;

    bool b_has_face_scan;

public:
    Face_config() :
        int_detail_level(2),
        dbl_mouth_shut_weight(0.0),
        dbl_eyelid_weight(0.0),
        dbl_ears_lm_weight(0.0),
        dbl_keep_body_vertices_weight(0.0),
        b_textured(true),
        b_provide_texture_manually(false),
        b_manual_eye_contour(false),
        b_has_face_scan(false)
    {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {
        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-f" && argv.size() > i+1)
            {
                dir_project = argv[i+1];
                if (dir_project.back() != '/')
                    dir_project.push_back('/');

                b_has_face_scan = true;
            }
        }

        if (!b_has_face_scan) return;

        find_key(data, "fn_pointset", fn_pointset, dir_project);
        find_key(data, "fn_invtrans_mesh", fn_invtrans_mesh, dir_project);
        find_key(data, "fn_photoscan_project", fn_photoscan_project, dir_project);
        find_key(data, "fn_texture", fn_texture, dir_project);
        find_key(data, "fn_frontal_image", fn_frontal_image, dir_project);
        find_key(data, "fn_contour_eyes", fn_contour_eyes, dir_project);
        find_key(data, "fn_ears_landmarks", fn_ears_landmarks, dir_project);
        find_key(data, "fn_cameras", fn_cameras, dir_project);
        find_key(data, "fn_landmarks_facialfeatures", fn_landmarks_facialfeatures, dir_project);

        find_key(data, "int_detail_level", int_detail_level);

        find_key(data, "dbl_mouth_shut_weight", dbl_mouth_shut_weight);
        find_key(data, "dbl_eyelid_weight", dbl_eyelid_weight);
        find_key(data, "dbl_ears_lm_weight", dbl_ears_lm_weight);
        find_key(data, "dbl_keep_body_vertices_weight", dbl_keep_body_vertices_weight);

        find_key(data, "b_textured", b_textured);
        find_key(data, "b_provide_texture_manually", b_provide_texture_manually);        
        find_key(data, "b_manual_eye_contour", b_manual_eye_contour);


    }
};

}

#endif
