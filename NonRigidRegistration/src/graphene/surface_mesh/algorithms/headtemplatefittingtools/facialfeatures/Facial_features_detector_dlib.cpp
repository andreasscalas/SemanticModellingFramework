#include "Facial_features_detector_dlib.h"


namespace graphene
{
namespace surface_mesh
{

Facial_features_detector_dlib::Facial_features_detector_dlib() :
    Facial_features_detector("dlib")
{

}

Facial_features_detector_dlib::~Facial_features_detector_dlib()
{

}

bool Facial_features_detector_dlib::load_model(const std::string &fn_model)
{
    dlib::deserialize(fn_model.c_str()) >> shape_predictor_;
    return shape_predictor_.num_features() > 0;
}

bool Facial_features_detector_dlib::detect(const std::string& fn_image)
{
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();

    if (shape_predictor_.num_features() == 0)
    {
        std::cerr << "Facial_features_detector_dlib::detect: Predictor not ready. Make sure the model is loaded!" << std::endl;
        return false;
    }

    dlib::array2d<dlib::rgb_pixel> img;

    dlib::load_image(img, fn_image);

    std::vector<dlib::rectangle> faces = detector(img);
    std::cout << "Facial_features_detector_dlib::detect: [STATUS] Number of faces detected: " << faces.size() << std::endl;

    dlib::full_object_detection shape;
    if (faces.size() > 0)
    {
         shape = shape_predictor_(img, faces[0]);
    }
    else
    {
        std::cout << "Facial_features_detector_dlib::detect: [ERROR] No face detected." << std::endl;
        return false;
    }

    facial_feature_coords_.clear();
    facial_feature_coords_.reserve(shape.num_parts());
    for (unsigned long i=0; i < shape.num_parts(); ++i)
    {
        const dlib::point & p = shape.part(i);
        facial_feature_coords_.push_back(Vec2i(p.x(), p.y()));

        //save eye contour features
        if (i >= 36 && i <= 47)
        {
            eye_feature_coords_.push_back(Vec2i(p.x(), p.y()));
        }
    }

    return ! facial_feature_coords_.empty();
}

}
}
