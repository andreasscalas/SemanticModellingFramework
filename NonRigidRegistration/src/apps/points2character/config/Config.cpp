
#include "Config.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace p2c
{

Config* Config::instance_ = nullptr;

Config* Config::instance()
{
    static Guard g;
    if (instance_ == nullptr)
    {
        instance_ = new Config;
    }
    return instance_;
}

Config::Config()
{
    //fetch environment variables

    const char* exec_env = getenv("PHOTOSCAN_EXEC");
    if (exec_env != NULL)
    {
        photoscan_exec_ = exec_env;
    }

}


bool Config::parse_cmd_line(int argc, char **argv)
{

    std::string tmp;
    int i;

    for (i=0; i < argc; ++i)
    {
        argv_.push_back(argv[i]);
    }

    for (size_t i=1; i < argv_.size(); ++i)
    {
        tmp = argv_[i];

        if (tmp.find("--") == 0)
        {
            if (tmp.find("--config=") == 0)
            {
                read_config_file(tmp.substr(9));
            }
        }
        else if (tmp[0] == '-' && argv_.size() > i+1)
        {
            if (tmp == "-c")
            {
                tmp = argv_[i+1];
                read_config_file(tmp);
            }

            ++i;
        }
        else
        {
            input_filenames_.push_back(tmp);
        }
    }

    //print_config_data("Full configuration:");

    if (data_.empty())
    {
        std::cerr << "Config::parse_cmd_line: [ERROR] Configuration is empty. Cannot proceed." << std::endl;
        return false;
    }

    //set photoscan script dir (it should be relative to the executable)
    photoscan_script_dir_ = argv_[0];
#ifdef _WIN32
    photoscan_script_dir_ = photoscan_script_dir_.substr(0, photoscan_script_dir_.find_last_of('\\')+1);
#else
    photoscan_script_dir_ = photoscan_script_dir_.substr(0, photoscan_script_dir_.find_last_of('/')+1);
#endif
    photoscan_script_dir_ += "scripts/photoscan/";

    general_config_.init(argv_, data_["general"]);
    tmpl_config_.init(argv_, data_["template"]);
    body_config_.init(argv_, data_["body"]);
    face_config_.init(argv_, data_["face"]);
    left_hand_config_.init(argv_, data_["lefthand"]);
    right_hand_config_.init(argv_, data_["righthand"]);

    print_config_data("\nConfig::parse_cmd_line: [WARNING] Unused configuration parameters:");

    return true;
}


bool Config::read_config_file(const std::string &filename)
{
    size_t fp;
    std::ifstream ifs;
    std::stringstream ss;
    std::vector<std::string> tokens;
    std::string cur_group;

    ifs.open(filename);


    std::string line;
    std::getline(ifs, line);

    do
    {
        std::getline(ifs, line);
        ss.clear();
        ss.str(line);


        if (line.empty() || line[0] == '#')
            continue;


        fp = line.find('[');
        if (fp != std::string::npos)
        {
            cur_group = line.substr(fp+1, line.find(']',fp+1)-1);
            continue;
        }

        std::map<std::string, std::string> &cur_group_map = data_[cur_group];

        split(line, '=', tokens);

        cur_group_map[tokens[0]] = tokens[1];
    }
    while(ifs.good());

/*
    //replace all
    for (std::map<std::string, std::string>::iterator it = data_.begin(); it != data_.end(); ++it)
    {
        std::string& value = it->second;
        size_t p=0;
        while ( (p = value.find("${",p) ) != std::string::npos)
        {
            //find size of string-to-replace ( -(p+2) since we want to skip '${' )
            const size_t string_size = value.find('}') - (p+2);
            //create substring of string-to-replace
            const std::string replace_key = value.substr(p+2, string_size);
            //find substracted key in key-value-map
            std::map<std::string, std::string>::iterator replace_it = data_.find(replace_key);

            if (replace_it != data_.end()) //found key in map, replace it!
            {
                const std::string &replace_string = replace_it->second;
                //replace in value; +3 since we have to take '$}' and '}' into account
                value.replace(p, string_size+3, replace_string);
                p += replace_string.size();
            }
            else //could not find key; not in config!
            {
                std::cerr << "Config::read_config_file: [WARNING] Could not replace \"" << replace_key << "\"." << std::endl;
                p += 2;
            }
        }
    }
*/
    ifs.close();

    return true;
}

void Config::print_config_data(const std::string& msg)
{
    bool something_to_print = false;
    for (auto d1: data_)
    {
        if (! d1.second.empty())
        {
            something_to_print = true;
            break;
        }
    }

    if (something_to_print)
    {
        std::cout << msg << std::endl;
        for (auto d1: data_)
        {
            if (! d1.second.empty())
            {
                std::cout << "[" << d1.first << "]" << std::endl;
                for (auto d2: d1.second)
                {
                    std::cout << d2.first << "=" << d2.second << "\n";
                }
                std::cout << std::endl;
            }
        }
    }

}

void Config::split(const std::string &in, const char delim, std::vector<std::string> &out)
{
    std::stringstream ss(in);
    std::string token;
    out.clear();

    while(std::getline(ss, token, delim))
    {
        out.push_back(token);
    }
}

}
