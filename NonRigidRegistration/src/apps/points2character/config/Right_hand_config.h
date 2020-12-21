#ifndef RIGHT_HAND_CONFIG_H
#define RIGHT_HAND_CONFIG_H

#include "Base_config.h"

namespace p2c
{

struct Right_hand_config : public Base_config
{
    std::string dir_project;
    std::string fn_pointset;
    std::string fn_landmarks;
    std::string fn_invtrans_mesh;
    std::string fn_photoscan_project;
    std::string fn_texture;

    bool b_landmarks_only_before_finefit;
    bool b_textured;
    bool b_provide_texture_manually;
    bool b_repair_fingertips_texture;

    bool b_has_right_hand_scan;

    int int_detail_level;

    Right_hand_config() :
        b_landmarks_only_before_finefit(true),
        b_textured(true),
        b_provide_texture_manually(false),
        b_has_right_hand_scan(false),
        int_detail_level(2)
    {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {
        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-r" && argv.size() > i+1)
            {
                dir_project = argv[i+1];
                if (dir_project.back() != '/')
                    dir_project.push_back('/');

                b_has_right_hand_scan = true;
            }
        }

        if (!b_has_right_hand_scan) return;

        find_key(data, "fn_pointset", fn_pointset, dir_project);
        find_key(data, "fn_landmarks", fn_landmarks, dir_project);
        find_key(data, "fn_invtrans_mesh", fn_invtrans_mesh, dir_project);
        find_key(data, "fn_photoscan_project", fn_photoscan_project, dir_project);
        find_key(data, "fn_texture", fn_texture, dir_project);

        find_key(data, "b_landmarks_only_before_finefit", b_landmarks_only_before_finefit);
        find_key(data, "b_textured", b_textured);
        find_key(data, "b_provide_texture_manually", b_provide_texture_manually);        
        find_key(data, "b_repair_fingertips_texture", b_repair_fingertips_texture);

        find_key(data, "int_detail_level", int_detail_level);

    }

};

}


#endif
