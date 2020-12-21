#ifndef FACIAL_FEATURES_DETECTOR_H
#define FACIAL_FEATURES_DETECTOR_H

#include <vector>
#include <graphene/geometry/Vector.h>

namespace graphene
{
namespace surface_mesh
{

class Facial_features_detector
{
public:
    Facial_features_detector(const std::string& name) :
        name_(name)
    {}

    ~Facial_features_detector(){}

    const std::string& get_name() {return name_;}

    virtual bool load_model(const std::string&)  {return true;}

    virtual bool detect(const std::string& fn_image) = 0;

    const std::vector<Vec2i> &facial_features() { return facial_feature_coords_;}

    bool save_facial_features(const std::string& filename);
    bool save_eye_features(const std::string& filename);

private:

    bool save_features(const std::vector<Vec2i> coords, const std::string& filename);

private:

    std::string name_;

protected:
    std::vector<Vec2i> facial_feature_coords_;

    std::vector<Vec2i> eye_feature_coords_;

};

}
}


#endif
