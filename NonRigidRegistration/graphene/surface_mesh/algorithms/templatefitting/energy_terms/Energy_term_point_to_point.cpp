//== INCLUDES =================================================================


#include "Energy_term_point_to_point.h"

#include <graphene/geometry/bary_coord.h>
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_point_to_point::
Energy_term_point_to_point(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Point to Point")
{
}


//-----------------------------------------------------------------------------


Energy_term_point_to_point::
~Energy_term_point_to_point()
{}


//-----------------------------------------------------------------------------

void
Energy_term_point_to_point::
pre_minimize_action()
{
    skin_correspondences(energy_term_data_.template_mesh, energy_term_data_.correspondences, skinned_correspondences_, false);
}

//-----------------------------------------------------------------------------


double
Energy_term_point_to_point::
evaluate()
{
    //choose either the skinnend correspondences or the "normal" ones based on whether skinned correspondences exist or not
    const std::vector<Correspondence>& correspondences = skinned_correspondences_.empty() ? energy_term_data_.correspondences : skinned_correspondences_;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;



    auto template_points              = template_mesh->get_vertex_property<Point>("v:point");
    auto templ_points_after_comp_corr = template_mesh->get_vertex_property<Point>("v:point_compcorr");


// BEGIN --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK
/*
    double sigma_hat=1.0;
    if ( (robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE) )
    {
        // compute median
        // see: <http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155>
        std::vector<double> res_vec_HACK;
        for (auto corr : correspondences)
        {
            if ( corr.constr_dir == corresp_dir_ps2mesh )
            {
                auto fvit = template_mesh->vertices(corr.face);
                const auto v0 = *fvit;
                const auto v1 = *(++fvit);
                const auto v2 = *(++fvit);
                const Point p0 = templ_points_after_comp_corr[v0];
                const Point p1 = templ_points_after_comp_corr[v1];
                const Point p2 = templ_points_after_comp_corr[v2];
                // get barycentric coordinates
                const Point b = graphene::geometry::barycentric_coordinates(corr.on_template, p0, p1, p2);
                Point new_x_pos = b[0]*template_points[v0] + b[1]*template_points[v1] + b[2]*template_points[v2];

                double u = norm( new_x_pos - corr.on_ps );

                if ( u != 0.0 )
                {
                    res_vec_HACK.push_back( u );
                }
                else
                {
                    std::cerr << " u is 0  (EVAL POI2POI ENERGY)  !!!" << std::endl;
//                exit(1);
                }
            }
        }

        double median_HACK = 0;
        {
            size_t n = res_vec_HACK.size() / 2;
            std::nth_element(res_vec_HACK.begin(), res_vec_HACK.begin()+n, res_vec_HACK.end());
            median_HACK = res_vec_HACK[n];
        }
        std::cerr << "   !!!!!!!!!!!!!!!!!!!!!!!!!!!!            median_HACK (IN EVAL FUNC.): " << median_HACK << std::endl;
        // compute sigma_hat
        sigma_hat = median_HACK; // / 0.675;
    }
*/
// END   --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK


    double error_match_poi2poi = 0.0;
    double sum_weights         = 0.0;
    for (auto corr : correspondences)
    {
        if ( corr.constr_dir == corresp_dir_mesh2ps )
        {
            double u = norm( template_points[corr.on_template_vh] - corr.on_ps );
            if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
            {
                error_match_poi2poi += corr.weight * u * u;
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
            {
                error_match_poi2poi += corr.weight * std::pow( u, robustness_parameters_.parameter );
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
            {
                double huber_thr = robustness_parameters_.parameter;
                if ( u > huber_thr )
                {
                    error_match_poi2poi += corr.weight * ( huber_thr*u - huber_thr*huber_thr/2.0 );
                }
                else
                {
                    error_match_poi2poi += corr.weight * ( u*u/2.0 );
                }
            }
/*
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
            {
                double scale_parameter = robustness_parameters_.parameter;
//                    error_match_poi2poi += corr.weight * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
                const double aux = u/sigma_hat;
                error_match_poi2poi += corr.weight * ( (aux*aux/2.0) / ( 1.0 + (aux*aux/(scale_parameter*scale_parameter)) ) ); // VERSION 3
            }
*/
            else // default to L2
            {
                error_match_poi2poi += corr.weight * u * u;
            }
        }
        else if ( corr.constr_dir == corresp_dir_ps2mesh )
        {
            auto fvit = template_mesh->vertices(corr.face);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point p0 = templ_points_after_comp_corr[v0];
            const Point p1 = templ_points_after_comp_corr[v1];
            const Point p2 = templ_points_after_comp_corr[v2];
            // get barycentric coordinates
            const Point b = graphene::geometry::barycentric_coordinates(corr.on_template, p0, p1, p2);
            Point new_x_pos = b[0]*template_points[v0] + b[1]*template_points[v1] + b[2]*template_points[v2];

            double u = norm( new_x_pos - corr.on_ps );
            if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
            {
                error_match_poi2poi += corr.weight * u * u;
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
            {
                error_match_poi2poi += corr.weight * std::pow( u, robustness_parameters_.parameter );
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
            {
                double huber_thr = robustness_parameters_.parameter;
                if ( u > huber_thr )
                {
                    error_match_poi2poi += corr.weight * ( huber_thr*u - huber_thr*huber_thr/2.0 );
                }
                else
                {
                    error_match_poi2poi += corr.weight * ( u*u/2.0 );
                }
            }
/*
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
            {
                double scale_parameter = robustness_parameters_.parameter;
//                    error_match_poi2poi += corr.weight * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
                error_match_poi2poi += corr.weight * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(sigma_hat*sigma_hat * scale_parameter*scale_parameter)) ) ) ) );
            }
*/
            else // default to L2
            {
                error_match_poi2poi += corr.weight * u * u;
            }
        }
        sum_weights         += corr.weight;
    }
    error_match_poi2poi *= (weight_ / sum_weights); // TODO evtl. sum_weights aus energy_term_data nehmen ?!

    return error_match_poi2poi;
}


//-----------------------------------------------------------------------------


void
Energy_term_point_to_point::
add_rows_small_lse( std::vector<Tripl>& coeffs,
                    Eigen::MatrixXd& B,
                    unsigned int& r)
{
    //std::cout << "[INFO] Energy_term_point_to_point::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    //choose either the skinnend correspondences or the "normal" ones based on whether skinned correspondences exist or not
    const std::vector<Correspondence>& correspondences = skinned_correspondences_.empty() ? energy_term_data_.correspondences : skinned_correspondences_;
    const unsigned int offset_match = column_offsets.offset_match;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
  
  
  
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_point_to_point::add_rows_small_lgs(...)'" << std::endl;
#endif

    auto templ_points_after_comp_corr  = template_mesh->get_vertex_property<Point>("v:point_compcorr");

    const double match_weight = sqrt(weight_ / energy_term_data_.sum_constr_weights);
    for (unsigned int i = 0; i < correspondences.size(); ++i)
    {
        const Correspondence& c = correspondences[i];

        const double cWeight = sqrt(c.weight);

        if ( correspondences[i].constr_dir == corresp_dir_mesh2ps )
        {
            // get vertex handle of sample
            const Surface_mesh::Vertex& vh = c.on_template_vh;

            // right hand side
            for (unsigned int j = 0; j < 3; ++j)
            {
                B(r, j) = match_weight * cWeight * c.on_ps[j];
            }

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + vh.idx(), cWeight * match_weight) );
            ++r;
        }
        else if ( correspondences[i].constr_dir == corresp_dir_ps2mesh )
        {
            // get vertices of closest triangle
            auto fvit = template_mesh->vertices(c.face);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point p0 = templ_points_after_comp_corr[v0];
            const Point p1 = templ_points_after_comp_corr[v1];
            const Point p2 = templ_points_after_comp_corr[v2];

            // get barycentric coordinates
            const Point b = graphene::geometry::barycentric_coordinates(c.on_template, p0, p1, p2);

            // right hand side
            for (unsigned int j = 0; j < 3; ++j)
            {
                B(r, j) = match_weight * cWeight * c.on_ps[j];
            }

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + v0.idx(), cWeight * b[0] * match_weight) );
            coeffs.push_back( Tripl(r, offset_match + v1.idx(), cWeight * b[1] * match_weight) );
            coeffs.push_back( Tripl(r, offset_match + v2.idx(), cWeight * b[2] * match_weight) );
            ++r;
        }
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_point_to_point::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)

{
    //std::cout << "[INFO] Energy_term_point_to_point::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    //choose either the skinnend correspondences or the "normal" ones based on whether skinned correspondences exist or not
    const std::vector<Correspondence>& correspondences = skinned_correspondences_.empty() ? energy_term_data_.correspondences : skinned_correspondences_;
    const unsigned int offset_match = column_offsets.offset_match;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

  
  
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_point_to_point::add_rows_big_lgs(...)'" << std::endl;
#endif

    auto templ_points_after_comp_corr  = template_mesh->get_vertex_property<Point>("v:point_compcorr");

    const double weight_match_poi2poi = sqrt( weight_ / energy_term_data_.sum_constr_weights );
    for (unsigned int i = 0; i < correspondences.size(); ++i)
    {
        // get closest point
        const Point& cp = correspondences[i].on_ps;

        const double cWeight = sqrt(correspondences[i].weight);

        if ( correspondences[i].constr_dir == corresp_dir_mesh2ps )
        {
            // get vertex handle of sample
            const Surface_mesh::Vertex& vh = correspondences[i].on_template_vh;

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[0];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx(), cWeight * weight_match_poi2poi) ); // z_i
            ++r;

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[1];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx() + 1, cWeight * weight_match_poi2poi) ); // z_i
            ++r;

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[2];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx() + 2, cWeight * weight_match_poi2poi) ); // z_i
            ++r;
        }
        else if ( correspondences[i].constr_dir == corresp_dir_ps2mesh )
        {
            // get vertices of closest triangle
            auto fvit = template_mesh->vertices(correspondences[i].face);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point p0 = templ_points_after_comp_corr[v0];
            const Point p1 = templ_points_after_comp_corr[v1];
            const Point p2 = templ_points_after_comp_corr[v2];

            // get barycentric coordinates
            const Point bc = graphene::geometry::barycentric_coordinates(correspondences[i].on_template, p0, p1, p2);

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[0];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx(), cWeight * weight_match_poi2poi * bc[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx(), cWeight * weight_match_poi2poi * bc[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx(), cWeight * weight_match_poi2poi * bc[2]) );
            ++r;

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[1];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx() + 1, cWeight * weight_match_poi2poi * bc[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx() + 1, cWeight * weight_match_poi2poi * bc[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx() + 1, cWeight * weight_match_poi2poi * bc[2]) );
            ++r;

            // right hand side
            b(r) = cWeight * weight_match_poi2poi * cp[2];

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx() + 2, cWeight * weight_match_poi2poi * bc[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx() + 2, cWeight * weight_match_poi2poi * bc[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx() + 2, cWeight * weight_match_poi2poi * bc[2]) );
            ++r;
        }
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_point_to_point::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_point_to_point::n_rows_small_sle()" << std::endl;

    return (unsigned int)energy_term_data_.correspondences.size();
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_point_to_point::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_point_to_point::n_rows_big_sle()" << std::endl;

    return (unsigned int)3*energy_term_data_.correspondences.size();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
