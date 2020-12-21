#ifndef FACIAL_FEATURES_DETECTOR_DLIB_H
#define FACIAL_FEATURES_DETECTOR_DLIB_H

#include "Facial_features_detector.h"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>

namespace graphene
{
namespace surface_mesh
{

class Facial_features_detector_dlib : public Facial_features_detector
{
public:
    Facial_features_detector_dlib();

    ~Facial_features_detector_dlib();

    bool load_model(const std::string &fn_model);

    virtual bool detect(const std::string& fn_image);

private:

    dlib::shape_predictor shape_predictor_;
};

}
}


#endif
