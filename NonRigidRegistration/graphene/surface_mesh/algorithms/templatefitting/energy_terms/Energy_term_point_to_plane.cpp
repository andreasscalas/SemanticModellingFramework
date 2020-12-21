//== INCLUDES =================================================================


#include "Energy_term_point_to_plane.h"

#include <graphene/geometry/bary_coord.h>

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_point_to_plane::
Energy_term_point_to_plane(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Point to Plane")
{}


//-----------------------------------------------------------------------------


Energy_term_point_to_plane::
~Energy_term_point_to_plane()
{}


//-----------------------------------------------------------------------------


double
Energy_term_point_to_plane::
evaluate()
{
    const std::vector<Correspondence>& correspondences = energy_term_data_.correspondences;
    const Point_set*    point_set     = energy_term_data_.point_set;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

  
    // TODO: check if property exists?
    auto template_points              = template_mesh->get_vertex_property<Point>("v:point");
    auto templ_points_after_comp_corr = template_mesh->get_vertex_property<Point>("v:point_compcorr");

    double error_match_poi2pla = 0.0;
    double sum_weights         = 0.0;
    for (auto corr : correspondences)
    {
        if ( corr.constr_dir == corresp_dir_mesh2ps )
        {
            error_match_poi2pla += corr.weight * std::pow( dot( corr.on_ps_n , template_points[corr.on_template_vh] - corr.on_ps) , 2 );
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
            error_match_poi2pla += corr.weight * std::pow( dot( corr.on_ps_n , new_x_pos - corr.on_ps) , 2 );
        }
        sum_weights += corr.weight;
    }
    error_match_poi2pla *= (weight_/ sum_weights); // TODO VLLT VON ENERGYTERMDATA NEHMEN ?!

    return error_match_poi2pla;
}


//-----------------------------------------------------------------------------


void
Energy_term_point_to_plane::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const std::vector<Correspondence>& correspondences = energy_term_data_.correspondences;
    const unsigned int offset_match = column_offsets.offset_match;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
  
  
  
  
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_point_to_plane::add_rows_big_lgs(...)'" << std::endl;
#endif

    auto templ_points_after_comp_corr  = template_mesh->get_vertex_property<Point>("v:point_compcorr");

    const double weight_match_poi2pla = sqrt(weight_ / energy_term_data_.sum_constr_weights);
    for (unsigned int i = 0; i < correspondences.size(); ++i)
    {
        // get closest point
        const Point& cp = correspondences[i].on_ps;

        // get normal of closest point
        const Normal& cpn = correspondences[i].on_ps_n;
//        const Normal& cpn = correspondences[i].on_template_n;

        const double cWeight = sqrt(correspondences[i].weight);

        if ( correspondences[i].constr_dir == corresp_dir_mesh2ps )
        {
            // right hand side
            b(r) = cWeight * weight_match_poi2pla * dot(cpn, cp);

            // get vertex handle of sample
            const Surface_mesh::Vertex& vh = correspondences[i].on_template_vh;

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx()    , cWeight * weight_match_poi2pla * cpn[0]) ); // z_i
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx() + 1, cWeight * weight_match_poi2pla * cpn[1]) ); // z_i
            coeffs.push_back( Tripl(r, offset_match + 3*vh.idx() + 2, cWeight * weight_match_poi2pla * cpn[2]) ); // z_i
            ++r;
        }
        else if ( correspondences[i].constr_dir == corresp_dir_ps2mesh )
        {
            // right hand side
            b(r) = cWeight * weight_match_poi2pla * dot(cpn, cp);

            // get vertices of closest triangle
            auto fvit = template_mesh->vertices(correspondences[i].face);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point p0 = templ_points_after_comp_corr[v0];
            const Point p1 = templ_points_after_comp_corr[v1];
            const Point p2 = templ_points_after_comp_corr[v2];

            // get barycentric coordinates
            const Point b = graphene::geometry::barycentric_coordinates(correspondences[i].on_template, p0, p1, p2);

            // begin_row
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx()    , cWeight * weight_match_poi2pla * b[0] * cpn[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx()    , cWeight * weight_match_poi2pla * b[1] * cpn[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx()    , cWeight * weight_match_poi2pla * b[2] * cpn[0]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx() + 1, cWeight * weight_match_poi2pla * b[0] * cpn[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx() + 1, cWeight * weight_match_poi2pla * b[1] * cpn[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx() + 1, cWeight * weight_match_poi2pla * b[2] * cpn[1]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v0.idx() + 2, cWeight * weight_match_poi2pla * b[0] * cpn[2]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v1.idx() + 2, cWeight * weight_match_poi2pla * b[1] * cpn[2]) );
            coeffs.push_back( Tripl(r, offset_match + 3*v2.idx() + 2, cWeight * weight_match_poi2pla * b[2] * cpn[2]) );
            ++r;
        }
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_point_to_plane::
n_rows_big_sle()
{
    return (unsigned int)energy_term_data_.correspondences.size();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
