#ifndef HEAD_CONFIG_H
#define HEAD_CONFIG_H

#include "Base_config.h"

namespace p2c
{

struct Head_config : public Base_config
{
    std::string dir_project;
    std::string fn_pointset;
    std::string fn_ears_landmarks;
    std::string fn_invtrans_mesh;
    std::string fn_frontal_image;
    std::string fn_cameras;
    std::string fn_landmarks_facialfeatures;

    double dbl_mouth_shut_weight;
    double dbl_ears_lm_weight;
    double dbl_keep_vertices_weight;

    int int_detail_level;

    Head_config() :
        int_detail_level(2),
        dbl_mouth_shut_weight(0.0),
        dbl_ears_lm_weight(0.0),
        dbl_keep_vertices_weight(0.0)
    {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {
        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-h" && argv.size() > i+1)
            {
                dir_project = argv[i+1];
                if (dir_project.back() != '/')
                    dir_project.push_back('/');
            }
        }

        find_key(data, "fn_pointset", fn_pointset, dir_project);
        find_key(data, "fn_ears_landmarks", fn_ears_landmarks, dir_project);
        find_key(data, "fn_invtrans_mesh", fn_invtrans_mesh, dir_project);
        find_key(data, "fn_frontal_image", fn_frontal_image, dir_project);
        find_key(data, "fn_cameras", fn_cameras, dir_project);
        find_key(data, "fn_landmarks_facialfeatures", fn_landmarks_facialfeatures, dir_project);

        find_key(data, "dbl_mouth_shut_weight", dbl_mouth_shut_weight);
        find_key(data, "dbl_ears_lm_weight", dbl_ears_lm_weight);
        find_key(data, "dbl_keep_vertices_weight", dbl_keep_vertices_weight);

        find_key(data, "int_detail_level", int_detail_level);

    }

};

}


#endif
