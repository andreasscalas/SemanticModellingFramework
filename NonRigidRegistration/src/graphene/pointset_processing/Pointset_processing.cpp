#include "Pointset_processing.h"
#include "Subsampling.h"


#include <graphene/surface_mesh/eigen_algorithms/surface_mesh_tools/misc.h>

#include <random>
#include <chrono>




void Pointset_processing::subsampling_according_to_mesh_meanedge(const surface_mesh::Surface_mesh& mesh, double meanedge_factor)
{
    geometry::Subsampling subsampling(pointset_node_.point_set());

    std::cout << "pointset_processing::subsampling_according_to_mesh_meanedge: [STATUS] #points before subsampling: " << pointset_node_.point_set().points().size() << std::endl;

    backup_landmarks(pointset_node_.point_set());

    subsampling.subsample(meanedge_factor * surface_mesh::mean_edge_length(mesh));

    restore_landmarks(pointset_node_.point_set());

    std::cout << "pointset_processing::subsampling_according_to_mesh_meanedge: [STATUS] #points after  subsampling: " << pointset_node_.point_set().points().size() << std::endl;
}


void Pointset_processing::replace_biggest_plane_ransac(float ymin, float ymax)
{
    backup_landmarks(pointset_node_.point_set());

    geometry::Point_set &ps = pointset_node_.point_set();

    std::vector<unsigned int> indices;

    //find points in the range ymin ymax and save their indices
    for (size_t i=0; i < ps.points().size(); ++i)
    {
        const Vec3f& point = ps.points()[i];
        if (point.y >= ymin && point.y <= ymax)
        {
            indices.push_back((unsigned int)i);
        }
    }

    //random number generator
    std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> distr(0,indices.size());

    const size_t max_it = 1000;
    const float model_threshold = 0.005f;

    int best_n_inliers=0;
    Vec4f best_coeff(0.0f);

    Vec4f coeff;
    for (size_t i=0; i < max_it && i < indices.size(); ++i)
    {
        //take three samples to compute plane coefficients
        //TODO: smarter choice of samples?
        int s1,s2,s3;
        do
        {
            s1 = distr(rng);
            s2 = distr(rng);
            s3 = distr(rng);
        }
        while (s1 == s2 || s2 == s3 || s1 == s3);

        const Vec3f& p1 = ps.points()[indices[(s1)]];
        const Vec3f& p2 = ps.points()[indices[(s2)]];
        const Vec3f& p3 = ps.points()[indices[(s3)]];

        //TODO: check for collinearity?

        const Vec3f e1 = (p2 - p1);
        const Vec3f e2 = (p2 - p3);

        //plane equation: ax + by + cz + d = 0;

        //coeff a,b,c: "normal" of plane
        const Vec3f n = normalize(cross(e1,e2));

        // coeff d: distance to plane
        const float d = -(n[0] * p1[0] + n[1] * p1[0] + n[2] * p1[2]);
        coeff[0] =  n[0];
        coeff[1] =  n[1];
        coeff[2] =  n[2];
        coeff[3] =  d;

        //check model quality
        int n_inliers=0;
        for (size_t i=0; i < indices.size(); ++i)
        {
            const Vec3f &p = ps.points()[indices[i]];

            //point to plane distance
            const Vec4f p_hc(p, 1.0f);

            const float dist = std::fabs(dot(p_hc, coeff));

            if (dist < model_threshold)
            {
                ++n_inliers;
            }
        }

        //is it better than the currently best model (criterion: number of inliers)
        if (n_inliers > best_n_inliers)
        {
            best_n_inliers = n_inliers;
            best_coeff = coeff;
        }
    }

    std::vector<size_t> inliers;
    //determine inliers with best model
    for (size_t i=0; i < indices.size(); ++i)
    {
        const Vec3f &p = ps.points()[indices[i]];

        //point to plane distance
        const Vec4f p_hc(p, 1.0f);
        const float dist = std::fabs(dot(p_hc, best_coeff));

        if (dist < model_threshold)
        {
            inliers.push_back((size_t)indices[i]);
        }
    }


    pointset_node_.set_selection(inliers);
    pointset_node_.delete_selected();

    const Vec4f ce = best_coeff;

    const Vec3f floor_normal = normalize( Vec3f(ce[0],ce[1], ce[2]) * ce[3] );

    float a,b;
    Point pe;
    for (a=-0.5f; a < 0.5f; a += 0.01f)
    {
        for (b=-0.5f; b < 0.5f; b += 0.01f)
        {
            pe[0] = a;
            pe[2] = b;
            pe[1] = (-ce[3] - ce[0]*a - ce[2]*b) * 1.0f/ce[1];
            pointset_node_.point_set().points_.push_back(pe);
            if (!pointset_node_.point_set().colors_.empty())
                pointset_node_.point_set().colors_.push_back(Color(0.0f,1.0f,0.0f));
            if (!pointset_node_.point_set().normals_.empty())
                pointset_node_.point_set().normals_.push_back(floor_normal);
        }
    }

    restore_landmarks(pointset_node_.point_set());

    pointset_node_.update_point_set();

}


void Pointset_processing::backup_landmarks(geometry::Point_set &point_set)
{
    if (point_set.lm_ears_.empty() && point_set.landmarks_.empty())
        return;

    lm_backup_points_.clear();
    lm_backup_normals_.clear();
    lm_backup_colors_.clear();
    lm_backup_ear_points_.clear();
    lm_backup_ear_normals_.clear();
    lm_backup_ear_colors_.clear();


    for (size_t i = 0; i < point_set.landmarks_.size(); ++i)
    {
        lm_backup_points_.push_back( point_set.points_[ point_set.landmarks_[i] ] );
        lm_backup_normals_.push_back( point_set.normals_[ point_set.landmarks_[i] ] );
        if (! point_set.colors_.empty())
        {
            lm_backup_colors_.push_back( point_set.colors_[ point_set.landmarks_[i] ] );
        }
    }
    point_set.landmarks_.clear();

    for (size_t i = 0; i < point_set.lm_ears_.size(); ++i)
    {
        lm_backup_ear_points_.push_back( point_set.points_[ point_set.lm_ears_[i] ] );
        lm_backup_ear_normals_.push_back( point_set.normals_[ point_set.lm_ears_[i] ] );
        if (! point_set.colors_.empty())
        {
            lm_backup_ear_colors_.push_back( point_set.colors_[ point_set.landmarks_[i] ] );
        }
    }
    point_set.lm_ears_.clear();
    point_set.selected_indices_.clear();
}

void Pointset_processing::restore_landmarks(geometry::Point_set &point_set)
{
    // add previous landmarks to new subsampled pointset
    size_t new_ps_size_wo_lm = point_set.points_.size();
    for (size_t i = 0; i < lm_backup_points_.size(); ++i)
    {
        point_set.points_.push_back( lm_backup_points_[i] );
        point_set.normals_.push_back( lm_backup_normals_[i] );
        if (! point_set.colors_.empty())
        {
            point_set.colors_.push_back(lm_backup_colors_[i]);
        }

        point_set.landmarks_.push_back( new_ps_size_wo_lm + i );
        point_set.selected_indices_.push_back( new_ps_size_wo_lm + i );
    }

    new_ps_size_wo_lm = point_set.points_.size();
    for (size_t i = 0; i < lm_backup_ear_points_.size(); ++i)
    {
        point_set.points_.push_back( lm_backup_ear_points_[i] );
        point_set.normals_.push_back( lm_backup_ear_normals_[i] );
        if (! point_set.colors_.empty())
        {
            point_set.colors_.push_back(lm_backup_ear_colors_[i]);
        }

        point_set.lm_ears_.push_back( new_ps_size_wo_lm + i );
        point_set.selected_indices_.push_back( new_ps_size_wo_lm + i );
    }
}

