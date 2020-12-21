#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>

namespace p2c
{

struct Base_config
{
    Base_config() {}

    virtual void init(const std::vector<std::string>& argv, std::map<std::string, std::string>& data) = 0;


protected:
    //helper

    template<class T>
    bool find_key(std::map<std::string, std::string>& data, const std::string &key, T &value)
    {
        std::map<std::string, std::string>::const_iterator cit = data.find(key);
        std::stringstream ss;

        if (cit != data.end())
        {
            ss << cit->second;
            ss >> value;
            data.erase(cit);
            return true;
        }
        else
        {
            std::cerr << "Base_config::find_key: [WARNING] Could not find key \"" << key << "\" in configuration. Using default value if possible." << std::endl;
            return false;
        }
    }

    template<class T>
    bool find_key(std::map<std::string, std::string>& data, const std::string &key, T &value, const std::string& prepend_string)
    {
        if (find_key(data, key, value))
        {
            const std::string tmp = value;
            value.clear();
            value = prepend_string + tmp;
            return true;
        }
        else
        {
            return false;
        }

    }

};

}


#endif

