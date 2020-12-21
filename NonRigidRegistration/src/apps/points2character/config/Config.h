#ifndef P2C_CONFIG_H
#define P2C_CONFIG_H


#include <string>
#include <vector>
#include <map>
#include "Template_config.h"
#include "Face_config.h"
#include "Body_config.h"
#include "Left_hand_config.h"
#include "Right_hand_config.h"
#include "General_config.h"

namespace p2c
{


class Config
{
    //singleton stuff ---------------------------------------------------------
public:
    static Config* instance();


private:
    static Config* instance_;
    Config();
    Config(const Config&) {}
    ~Config(){}

    /*!
         * \brief The Guard class
         * Guard to make sure all data are properly deleted.
         */
    class Guard
    {
    public:
        ~Guard()
        {
            if (Config::instance_ != nullptr)
            {
                delete Config::instance_;
                Config::instance_ = nullptr;
            }
        }
    };

    friend class Guard;

    //singleton stuff end------------------------------------------------------

private:
    std::vector<std::string> argv_;

    //from environment
    std::string photoscan_exec_;
    std::string photoscan_script_dir_;

    std::vector<std::string> input_filenames_;


    std::map<std::string, std::map<std::string, std::string> > data_;

    Template_config tmpl_config_;

    Body_config body_config_;

    Face_config face_config_;

    Left_hand_config left_hand_config_;

    Right_hand_config right_hand_config_;

    General_config general_config_;
public:

    bool parse_cmd_line(int argc, char** argv);

    //getter
    const std::vector<std::string>& get_argv() { return argv_; }

    const std::vector<std::string>& input_filenames() { return input_filenames_; }

    Template_config &tmpl(){return tmpl_config_;}
    Face_config &face(){return face_config_;}
    Body_config &body(){return body_config_;}
    Left_hand_config &left_hand(){return left_hand_config_;}
    Right_hand_config &right_hand(){return right_hand_config_;}
    General_config & general(){return general_config_;}

    const std::string& get_photoscan_exec() {return photoscan_exec_;}
    const std::string& get_photoscan_script_dir() {return photoscan_script_dir_;}

private:

    bool read_config_file(const std::string&filename);

    void print_config_data(const std::string &msg);

    //parse helper
    void split(const std::string& in, const char delim, std::vector<std::string> &out);
};

}

#endif
