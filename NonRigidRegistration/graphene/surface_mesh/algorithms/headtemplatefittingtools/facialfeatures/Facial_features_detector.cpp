
#include "Facial_features_detector.h"

#include <fstream>

namespace graphene
{
namespace surface_mesh
{

bool Facial_features_detector::save_facial_features(const std::string &filename)
{
    return save_features(facial_feature_coords_, filename);
}

bool Facial_features_detector::save_eye_features(const std::string &filename)
{
    return save_features(eye_feature_coords_, filename);
}


bool Facial_features_detector::save_features(const std::vector<Vec2i> coords, const std::string &filename)
{

    std::ofstream ofs( filename );
    if(! ofs)
    {
        std::cerr << "[ERROR] Can't save 2d pixel-coordinates to file: " << filename << std::endl;
        return false;
    }

    for (size_t i(0); i < coords.size(); i++)
    {
        ofs << coords[i][0];
        ofs << " ";
        ofs << coords[i][1];
        ofs << std::endl;
    }
    ofs.close();

    return true;
}

}
}


