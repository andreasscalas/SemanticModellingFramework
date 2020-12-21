#ifndef FACIAL_FEATURES_DETECTION_H
#define FACIAL_FEATURES_DETECTION_H

#include <vector>
#include "Facial_features_detector.h"

#include <graphene/geometry/Point_set.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/utility/Agi_camera.h>

namespace graphene
{
namespace surface_mesh
{

class Facial_features_detection
{
private:

    struct Pointset_correspondence
    {
        size_t pointset_index;
        float  distance_to_landmark;
        float  distance_to_camera;
        bool   is_valid;
    };

public:
    enum FF_detectors
    {
        FFD_DLIB = 0,
        FFD_MAX
    };

public:
    Facial_features_detection();

    ~Facial_features_detection();

    bool load_model(const std::string& fn_model, FF_detectors type);

    bool detect_ff(
            const std::string &fn_image,
            FF_detectors type);

    bool map_ff_to_pointset(
            geometry::Point_set &pointset,
            const std::string &fn_image,
            const std::string &fn_cameras,
            FF_detectors type);

    bool save_eye_features(const std::string& filename, FF_detectors type);

private:

    std::vector<Facial_features_detector*> detectors_;

    bool find_landmarks_swenninger(const geometry::Point_set &pointset,
            const utility::Agi_camera &camera,
            const std::vector<Vec2i> &facial_features,
            std::vector<unsigned int> &out_landmarks
            );

    bool find_landmarks_twaltema(const geometry::Point_set &pointset,
            const utility::Agi_camera &camera,
            const std::vector<Vec2i> &facial_features,
            std::vector<unsigned int> &out_landmarks
            );

    float get_median(std::vector<float> samples);

};

}
}


#endif

