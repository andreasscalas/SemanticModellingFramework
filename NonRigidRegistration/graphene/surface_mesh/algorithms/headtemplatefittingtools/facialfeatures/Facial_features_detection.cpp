
#include "Facial_features_detection.h"
#include "Facial_features_detector_dlib.h"

#include <float.h>


namespace graphene
{
namespace surface_mesh
{

Facial_features_detection::Facial_features_detection()
{
    detectors_.resize(FFD_MAX, nullptr);

    detectors_[FFD_DLIB] = new Facial_features_detector_dlib();
}

Facial_features_detection::~Facial_features_detection()
{

}

bool Facial_features_detection::load_model(const std::string &fn_model, FF_detectors type)
{
    if(! detectors_[type]->load_model(fn_model))
    {
        std::cout << "Facial_features_detection::load_model: [ERROR] Could not load model for "
                  << detectors_[type]->get_name() << " detector with filename \""
                  << fn_model << "\"." << std::endl;
        return false;
    }
    return true;
}


bool Facial_features_detection::detect_ff(const std::string& fn_image,
        FF_detectors type)
{
    Facial_features_detector* ffd = detectors_[type];
    if (ffd == nullptr)
    {
        std::cerr << "Facial_features_detection::detect_ff_and_map_to_pointset: [ERROR] No detector of type " << type << "." << std::endl;
        return false;
    }

    if (! ffd->detect(fn_image))
    {
        std::cerr << "Facial_features_detection::detect_ff_and_map_to_pointset: [ERROR] Unable to detect facial features." << std::endl;
        return false;
    }
    return true;

}

bool Facial_features_detection::map_ff_to_pointset(
        geometry::Point_set &pointset,
        const std::string &fn_image,
        const std::string &fn_cameras,
        FF_detectors type)
{
    Facial_features_detector* ffd = detectors_[type];

    //load camera calibration
    utility::Agi_cameras cameras(fn_cameras);

    utility::Agi_camera *camera_of_image = cameras.get_camera_by_filename(fn_image);

    if (camera_of_image == nullptr)
    {
        std::cerr << "Facial_features_detection::detect_ff_and_map_to_pointset: [ERROR] Unable to find camera for image with filename: \"" << fn_image << "\"." << std::endl;
        return false;
    }

    find_landmarks_twaltema(pointset, *camera_of_image, ffd->facial_features(), pointset.landmarks_);

    //copy landmarks to selection for visualization
    pointset.selected_indices_ = pointset.landmarks_;

    return true;
}

bool Facial_features_detection::save_eye_features(const std::string &filename, FF_detectors type)
{
    Facial_features_detector* ffd = detectors_[type];
    if (ffd == nullptr)
    {
        std::cerr << "Facial_features_detection::detect_ff_and_map_to_pointset: [ERROR] No detector of type " << type << "." << std::endl;
        return false;
    }

    return ffd->save_eye_features(filename);
}

bool Facial_features_detection::find_landmarks_swenninger(
        const geometry::Point_set &point_set,
        const utility::Agi_camera &camera,
        const std::vector<Vec2i> &facial_features,
        std::vector<unsigned int> &out_landmarks)
{
    //
    // x,y coordinates of the detected landmarks in the image
    //
    // TODO: calculate (read from dlib, openface, or whatever tool)
    //
    std::vector<Vec2f> landmark_pixel_coordinates(facial_features.size());

    for (size_t i=0; i < facial_features.size(); ++i)
    {
        landmark_pixel_coordinates[i][0] = (float) facial_features[i][0];
        landmark_pixel_coordinates[i][1] = (float) facial_features[i][1];
    }


    //
    // Get closest 3D point on point_set for every landmark
    //

    // TODO: this is a randomly picked threshold
    float max_pixel_distance_for_landmark = 7;

    std::vector<Pointset_correspondence> pointset_landmarks;
    for (size_t landmark_index = 0; landmark_index < landmark_pixel_coordinates.size(); ++landmark_index)
    {
        //
        // Landmark position in image space
        //
        Vec2f& landmark = landmark_pixel_coordinates[landmark_index];

        //
        // The corresponding candidates in the pointset
        //
        std::vector<Pointset_correspondence> candidates;

        //
        // Add all candidates, who's distance (in pixels) to the landmark is less than max_pixel_distance_for_landmark
        //
//#pragma omp declare reduction (merge : std::vector<Pointset_correspondence> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
//#pragma omp parallel for reduction(merge : candidates)
        for (size_t point_index = 0; point_index < point_set.size(); ++point_index)
        {
            const Point &p = point_set.points_[point_index];
            float distance_to_camera = distance(p, camera.get_center());

            // Project point
            Vec2f projected = camera.world_to_pixel_swenninger(p);

            float distance_to_landmark = distance(projected, landmark);

            if (distance_to_landmark < max_pixel_distance_for_landmark)
            {
                candidates.push_back({point_index, distance_to_landmark, distance_to_camera, false});
            }
        }

        //
        // Get smallest distance to the camera center
        //
        float  smallest_distance_to_camera = FLT_MAX;
        for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
        {
            if (candidates[candidate_index].distance_to_camera < smallest_distance_to_camera)
            {
                smallest_distance_to_camera = candidates[candidate_index].distance_to_camera;
            }
        }

        Pointset_correspondence best_candidate = { 0, FLT_MAX, FLT_MAX, false };

        //
        // Of all the candidates, chose the one with the minimum distance to the camera center
        //

        //
        // TODO: this is a randomly chosen threshold
        //
        float  distance_3D_threshold = smallest_distance_to_camera + 0.1;
        for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
        {
            Pointset_correspondence& candidate = candidates[candidate_index];

            if (candidate.distance_to_landmark < best_candidate.distance_to_landmark &&
                    candidate.distance_to_camera   < distance_3D_threshold)
            {
                best_candidate.pointset_index        = candidate.pointset_index;
                best_candidate.distance_to_landmark  = candidate.distance_to_landmark;
                best_candidate.distance_to_camera    = candidate.distance_to_camera;
                best_candidate.is_valid = true;
            }
        }

        pointset_landmarks.push_back(best_candidate);
    }


    //
    // Median Absolute Deviation (MAD) filtering
    //     https://en.wikipedia.org/wiki/Median_absolute_deviation
    //
    std::vector<float> valid_3D_distances;
    for (auto candidate : pointset_landmarks)
    {
        if (candidate.is_valid)
            valid_3D_distances.push_back(candidate.distance_to_camera);
    }

    float median = get_median(valid_3D_distances);
    printf("Median 3D Dist: %f\n", median);

    std::vector<float> absolute_deviations;
    for (auto candidate : pointset_landmarks)
    {
        if (candidate.is_valid)
            absolute_deviations.push_back(std::abs(candidate.distance_to_camera - median));
    }

    float MAD_multiplier = 1.4826f;
    float median_absolute_deviation = MAD_multiplier * get_median(absolute_deviations);

    //
    // If the distance to the camera is more than 3 times the MAD, we ignore this candidate
    //
    for (size_t i = 0; i < pointset_landmarks.size(); ++i)
    {
        printf("Candidate #%lu: x-median: %f\n", i, pointset_landmarks[i].distance_to_camera - median);

        if (pointset_landmarks[i].is_valid)
        {
            if (std::abs(pointset_landmarks[i].distance_to_camera - median) > 3 * median_absolute_deviation)
            {
                pointset_landmarks[i].is_valid = false;
            }
        }
    }

    out_landmarks.clear();
    out_landmarks.resize(pointset_landmarks.size());
    for (size_t i=0; i < out_landmarks.size(); ++i)
    {
        out_landmarks[i] = pointset_landmarks[i].pointset_index;
        std::cout << out_landmarks[i] << std::endl;
    }

    return ! out_landmarks.empty();
}


bool Facial_features_detection::find_landmarks_twaltema(
        const geometry::Point_set &point_set,
        const utility::Agi_camera &camera,
        const std::vector<Vec2i> &facial_features,
        std::vector<unsigned int> &out_landmarks)
{

    out_landmarks.clear();


    const Vec3f cam_center = camera.get_center();
    const float max_pixel_dist = 20.0f;
    std::vector<unsigned int> candidates;

    for (size_t i=0; i < facial_features.size(); ++i)
    {
        const Vec2f ff((float)facial_features[i][0], (float)facial_features[i][1]);

        candidates.clear();


        //find candidates by pixel distance
        for (size_t j=0; j < point_set.points().size(); ++j)
        {
            const Vec2f point2D = camera.world_to_pixel(point_set.points()[j]);
            const float dist = norm(point2D - ff);

            if (dist < max_pixel_dist)
            {
                candidates.push_back((unsigned int) j);
            }
        }


        //use point closest to camera position
        float closest_dist2D = FLT_MAX;
        //float closest_dist3D = FLT_MAX;
        unsigned int closest_idx=0;
        for (size_t j=0; j < candidates.size(); ++j)
        {
            const Vec3f point3D = point_set.points()[candidates[j]];
            //const Vec2f point2D = camera.world_to_pixel(point_set.points()[candidates[j]]);

            //const float dist2D = norm(point2D - ff);
            const float dist3D = norm(point3D - cam_center);


            if (dist3D < closest_dist2D)
            {
                closest_dist2D = dist3D;
                //closest_dist3D = dist3D;
                closest_idx    = candidates[j];
            }
        }

        out_landmarks.push_back(closest_idx);
    }




    return out_landmarks.size() == facial_features.size();

}

// Copies input, because we sort it to find the median
float Facial_features_detection::get_median(std::vector<float> samples)
{
    if (samples.size() == 0) { return FLT_MAX; }

    std::sort(samples.begin(), samples.end(), [](float a, float b) -> bool { return a > b; } );

    if (samples.size() % 2 == 0)
    {
        int half = samples.size() / 2;
        return 0.5f * (samples[half] + samples[half-1]);
    }
    else
    {
        return samples[samples.size() / 2];
    }
}


}
}
