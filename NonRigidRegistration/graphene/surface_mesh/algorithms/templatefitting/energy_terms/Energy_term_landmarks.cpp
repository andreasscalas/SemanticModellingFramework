//== INCLUDES =================================================================


#include "Energy_term_landmarks.h"
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_landmarks::
Energy_term_landmarks(const Energy_term_data& energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Landmarks")
{
}


//-----------------------------------------------------------------------------


Energy_term_landmarks::
~Energy_term_landmarks()
{}

//-----------------------------------------------------------------------------

void
Energy_term_landmarks::
pre_minimize_action()
{
    skin_pointset_landmarks(energy_term_data_.template_mesh, energy_term_data_.point_set, energy_term_data_.landmarks_manager, skinned_landmarks_pointset_, false);
}

//-----------------------------------------------------------------------------

double
Energy_term_landmarks::
evaluate()
{
    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    const Point_set*    point_set     = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    if (!landmarks_manager.check_dimension(point_set, template_mesh))
    {
        return DBL_MAX;
    }

    std::vector<Point> lm_ps;
    landmarks_manager.get_landmarks_point_set_stdvec(point_set, lm_ps);
    std::vector<Point> lm_templ;
    landmarks_manager.get_landmarks_template_stdvec(template_mesh, lm_templ);

    //choose either the skinnend landmarks or the "normal" ones based on whether skinned landmarks exist or not
    const std::vector<Point>& landmarks_pointset = skinned_landmarks_pointset_.empty() ? lm_ps : skinned_landmarks_pointset_;


    Surface_mesh::Vertex_property<double> local_landmark_weight = template_mesh->get_vertex_property<double>("v:landmark_weight");
    const std::vector<Surface_mesh::Vertex>& landmark_prop = template_mesh->get_mesh_property<std::vector<Surface_mesh::Vertex> >("m:landmarks")[0];

    double error_match_landmarks = 0.0;
    double local_weight = 1.0;
    const unsigned int L = landmarks_pointset.size();
    for (unsigned int i = 0; i < L; ++i)
    {
        if (local_landmark_weight)
        {
            local_weight = local_landmark_weight[landmark_prop[i]];
            if (local_weight < 1e-5)
                continue;
            local_weight = sqrt(local_weight);
        }

        double u = norm( landmarks_pointset[i] - lm_templ[i] );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_match_landmarks += u * u * local_weight;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_match_landmarks += std::pow( u, robustness_parameters_.parameter ) * local_weight;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            std::cerr << "ERSTMAL NICHT " << std::endl; exit(1);
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_match_landmarks += ( huber_thr*u - huber_thr*huber_thr/2.0 ) * local_weight;
            }
            else
            {
                error_match_landmarks += ( u*u/2.0 ) * local_weight;
            }
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            std::cerr << "ERSTMAL NICHT " << std::endl; exit(1);
            double scale_parameter = robustness_parameters_.parameter;
//                error_match_landmarks += ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
            error_match_landmarks += ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ) * local_weight; // VERSION 3
        }
        else // default to L2
        {
            error_match_landmarks += u * u * local_weight;
        }
    }
    error_match_landmarks *= (weight_ / L);

    return error_match_landmarks;
}


//-----------------------------------------------------------------------------

void
Energy_term_landmarks::
add_rows_small_lse(
        std::vector<Tripl> &coeffs,
        Eigen::MatrixXd &B,
        unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_landmarks::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Point_set* point_set = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_landmarks::add_rows_small_lgs(...)'" << std::endl;
#endif

    if (!landmarks_manager.check_dimension(point_set, template_mesh))
    {
        return;
    }

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > lm_prop = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    const std::vector<Surface_mesh::Vertex>& tm_landmarks = lm_prop[0];

    double local_weight = 1.0;
    Surface_mesh::Vertex_property<double> local_landmark_weight = template_mesh->get_vertex_property<double>("v:landmark_weight");

    std::vector<Point> lm_ps;
    landmarks_manager.get_landmarks_point_set_stdvec(point_set, lm_ps);


    //choose either the skinnend landmarks or the "normal" ones based on whether skinned landmarks exist or not
    const std::vector<Point>& landmarks_pointset = skinned_landmarks_pointset_.empty() ? lm_ps : skinned_landmarks_pointset_;

    const unsigned int L = landmarks_pointset.size();
    const double landmarks_weight = sqrt(weight_ / L);
    for (unsigned int i = 0; i < L; ++i)
    {
        if (local_landmark_weight)
        {
            local_weight = local_landmark_weight[tm_landmarks[i]];
            if (local_weight < 1e-5)
            {
                std::cerr << "[TEMP] SHIT !!!" << std::endl; exit(1);
                continue;
            }
            local_weight = sqrt(local_weight);
        }

        const int idx = tm_landmarks[i].idx();
        const Point t = landmarks_pointset[i];

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = local_weight * landmarks_weight * t[j];
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx, local_weight * landmarks_weight) );
        ++r;
    }
}


//-----------------------------------------------------------------------------

void
Energy_term_landmarks::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_landmarks::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Landmarks_manager& landmarks_manager = energy_term_data_.landmarks_manager;
    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Point_set* point_set = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    if (!landmarks_manager.check_dimension(point_set, template_mesh))
    {
        return;
    }

    double local_weight = 1.0;
    Surface_mesh::Vertex_property<double> local_landmark_weight = template_mesh->get_vertex_property<double>("v:landmark_weight");

    std::vector<Surface_mesh::Vertex> tm_landmarks;
    landmarks_manager.get_indices_template(template_mesh, tm_landmarks);
    std::vector<Point> lm_ps;
    landmarks_manager.get_landmarks_point_set_stdvec(point_set, lm_ps);

    //choose either the skinnend landmarks or the "normal" ones based on whether skinned landmarks exist or not
    const std::vector<Point>& landmarks_pointset = skinned_landmarks_pointset_.empty() ? lm_ps : skinned_landmarks_pointset_;


    const unsigned int L = landmarks_pointset.size();

    const double weight_match_landmarks = sqrt(weight_ / L);
    for (unsigned int i = 0; i < L; ++i)
    {
        const int idx = tm_landmarks[i].idx();
        const Point t = landmarks_pointset[i];

        if (local_landmark_weight)
        {
            local_weight = local_landmark_weight[tm_landmarks[i]];
            if (local_weight < 1e-5)
            {
                continue;
            }
            local_weight = sqrt(local_weight);
        }

        // right hand side
        b(r) = local_weight * weight_match_landmarks * t[0];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx, local_weight * weight_match_landmarks) );
        ++r;

        // right hand side
        b(r) = local_weight * weight_match_landmarks * t[1];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 1, local_weight * weight_match_landmarks) );
        ++r;

        // right hand side
        b(r) = local_weight * weight_match_landmarks * t[2];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 2, local_weight * weight_match_landmarks) );
        ++r;
    }

}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_landmarks::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_landmarks::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    unsigned int c=0;
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > lm_prop = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    if (lm_prop)
    {
        const std::vector<Surface_mesh::Vertex>& tm_landmarks = lm_prop[0];
        Surface_mesh::Vertex_property<double> local_landmark_weight = template_mesh->get_vertex_property<double>("v:landmark_weight");

        if (local_landmark_weight)
        {
            for (size_t i=0; i < tm_landmarks.size(); ++i)
            {
                if (local_landmark_weight[tm_landmarks[i]] > 1e-5)
                {
                    ++c;
                }
            }
        }
        else
        {
            c = (unsigned int) tm_landmarks.size();
        }
    }

    return c;
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_landmarks::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_landmarks::n_rows_big_sle()" << std::endl;

    unsigned int c=0;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > lm_prop = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    if (lm_prop)
    {
        const std::vector<Surface_mesh::Vertex>& tm_landmarks = lm_prop[0];
        Surface_mesh::Vertex_property<double> local_landmark_weight = template_mesh->get_vertex_property<double>("v:landmark_weight");

        if (local_landmark_weight)
        {
            for (size_t i=0; i < tm_landmarks.size(); ++i)
            {
                if (local_landmark_weight[tm_landmarks[i]] > 1e-5)
                {
                    ++c;
                }
                else
                {
                    //std::cout << "landmark ignored " << tm_landmarks[i] << std::endl;
                }
            }
        }
        else
        {
            c = (unsigned int) tm_landmarks.size();
        }
    }

    return c*3;
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
