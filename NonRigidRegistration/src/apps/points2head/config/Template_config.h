#ifndef TEMPLATE_CONFIG_H
#define TEMPLATE_CONFIG_H

#include "Base_config.h"

namespace p2c
{
struct Template_config : public Base_config
{
    std::string dir_template_db;
    std::string fn_template_head;
    std::string fn_landmarks_face_ears;
    std::string fn_landmarks_facialfeatures;
    std::string fn_mapping_body2head;
    std::string fn_pca_head_mean_mesh;
    std::string fn_pca_head;
    std::string fn_vw_face_and_ears_fitting;
    std::string fn_sel_keep_vertices;
    std::string fn_sel_mouth_shut;
    std::string fn_sel_vertices_to_omit;
    std::string fn_facialfeatures_model;

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

        find_key(data, "fn_template_head", fn_template_head, dir_template_db);
        find_key(data, "fn_landmarks_face_ears", fn_landmarks_face_ears, dir_template_db);        
        find_key(data, "fn_landmarks_facialfeatures", fn_landmarks_facialfeatures, dir_template_db);
        find_key(data, "fn_mapping_body2head", fn_mapping_body2head, dir_template_db);
        find_key(data, "fn_pca_head_mean_mesh", fn_pca_head_mean_mesh, dir_template_db);
        find_key(data, "fn_pca_head", fn_pca_head, dir_template_db);
        find_key(data, "fn_vw_face_and_ears_fitting", fn_vw_face_and_ears_fitting, dir_template_db);
        find_key(data, "fn_sel_keep_vertices", fn_sel_keep_vertices, dir_template_db);
        find_key(data, "fn_sel_mouth_shut", fn_sel_mouth_shut, dir_template_db);
        find_key(data, "fn_sel_vertices_to_omit", fn_sel_vertices_to_omit, dir_template_db);
        find_key(data, "fn_facialfeatures_model", fn_facialfeatures_model, dir_template_db);
    }
};
}


#endif
