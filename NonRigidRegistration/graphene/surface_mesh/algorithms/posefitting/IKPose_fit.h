//=============================================================================
#ifndef GRAPHENE_IKPOSE_FIT_H
#define GRAPHENE_IKPOSE_FIT_H
//=============================================================================

//== INCLUDES =================================================================

#include "Pose_fit_data.h"
#include "ik/kinematics/Kinematics.h"
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>

//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {

using graphene::character::Character;
using graphene::geometry::Point_set;

//== CLASS DEFINITION =========================================================

class IKPose_fit
{
protected:

    Pose_fit_data* data_;

    ik::Skeleton* ik_skeleton_;
    std::vector<ik::Joint*> ik_joints_;

    ik::InverseKinematics* ik_solver_;

    int max_iterations_;
    double conv_value_;
    double max_normal_angle_;
    double max_dist_;
    double lambda_;

    bool use_landmarks_;
    bool use_correspondences_;

    bool is_initialized_;

    surface_mesh::Landmarks_manager landmarks_manager_;


    std::vector<Correspondence> correspondences_;
    Correspondences_settings correspondence_settings_;

    std::vector<Vec3f> targets_;
    std::vector<Vec3f> positions_;

    // helper for checking convergence
    int    iter_;
    double error_;
    double error_prev_;
    double rel_error_;

public:
    IKPose_fit(Pose_fit_data* data);
    ~IKPose_fit();

    bool start();

    void clean_up();

    bool minimization_step();

    void initial_aligment_with_pca(const std::string& filename_pca_mean, const std::string &filename_pca, double corr_distance, double corr_normal, int max_iter);


    void set_max_iterations(int iter) { max_iterations_ = iter; }
    void set_convergence_value(double conv_value) { conv_value_ = conv_value; }
    void set_max_normalangle(double angle) { max_normal_angle_ = angle; }
    void set_max_dist(double dist) { max_dist_ = dist; }

    void set_use_landmarks(bool enable) { use_landmarks_ = enable; }
    void set_use_correspondences(bool enable) { use_correspondences_ = enable; }

    void set_lambda(double lambda) { lambda_ = lambda; }

    bool is_converged() { return !(rel_error_ > conv_value_ && iter_ < max_iterations_); }

protected:


    bool init();

    bool minimize();

    //void minimize_feet();

    void clear();

    void build_ik_skeleton_recursive(character::Joint* joint);

    void create_mapping();

    void set_targets_and_positions(const std::vector<Vec3f>& targets, const std::vector<Vec3f>& positions);

    void solve();

    void copy_pose();

    void transform_points(const Mat4f& M);

    void transform_points(const Mat4f& M, const std::vector<unsigned int>& indices, std::vector<Point>& result);

};

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_IKPose_fit_H
//=============================================================================
