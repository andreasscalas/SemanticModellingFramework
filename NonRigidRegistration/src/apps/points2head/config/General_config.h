#ifndef GENERAL_CONFIG_H
#define GENERAL_CONFIG_H

#include "Base_config.h"

namespace p2c
{

struct General_config : Base_config
{
    std::string fn_result;

    bool b_hidewindow;

public:
    General_config () :
        b_hidewindow(false)
    {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data)
    {
        for (size_t i=0; i < argv.size(); ++i)
        {
            if (argv[i] == "-o" && argv.size() > i+1)
            {
                fn_result = argv[i+1];
            }
            else if (argv[i] == "--hidewindow")
            {
                b_hidewindow = true;
            }
        }
    }
};

}

#endif
