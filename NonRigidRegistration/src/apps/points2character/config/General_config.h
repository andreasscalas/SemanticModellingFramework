#ifndef GENERAL_CONFIG_H
#define GENERAL_CONFIG_H

#include "Base_config.h"

namespace p2c
{

struct General_config : Base_config
{
    std::string fn_result;

    bool b_adjust_eye_and_teeth_luminance;
    bool b_icspace_retargeting;
    bool b_convert_skeleton;
    bool b_hidewindow;
    bool b_adjust_iris_color;

    double dbl_result_height;

public:
    General_config () :
        dbl_result_height(1.80),
        b_icspace_retargeting(false),
        b_convert_skeleton(false),
        b_hidewindow(false),
        b_adjust_iris_color(false)
    {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {
        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-o" && argv.size() > i+1)
            {
                fn_result = argv[i+1];
            }
            else if (argv[i] == "-h" && argv.size() > i+1)
            {
                std::stringstream ss(argv[i+1]);
                ss >> dbl_result_height;
            }
            else if (argv[i] == "--hidewindow")
            {
                b_hidewindow = true;
            }
        }

        find_key(data, "b_adjust_eye_and_teeth_luminance", b_adjust_eye_and_teeth_luminance);
        find_key(data, "b_icspace_retargeting", b_icspace_retargeting);
        find_key(data, "b_convert_skeleton", b_convert_skeleton);
        find_key(data, "b_adjust_iris_color", b_adjust_iris_color);

    }
};

}

#endif
