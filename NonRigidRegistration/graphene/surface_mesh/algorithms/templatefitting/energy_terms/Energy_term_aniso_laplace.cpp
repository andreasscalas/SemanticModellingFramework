//== INCLUDES =================================================================


#include "Energy_term_aniso_laplace.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_aniso_laplace::
Energy_term_aniso_laplace(const Energy_term_data& energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Anisotropic Laplace")
{
}


//-----------------------------------------------------------------------------


Energy_term_aniso_laplace::
~Energy_term_aniso_laplace()
{}


//-----------------------------------------------------------------------------


double
Energy_term_aniso_laplace::
evaluate()
{
    double surface_area_aniso = energy_term_data_.surface_area_aniso;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points        = template_mesh->get_vertex_property<Point>("v:point");
    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");

    auto templ_local_weight     = template_mesh->get_vertex_property<double>("v:regularization_weight");
    double local_weight = 1.0;

    auto area_prop        = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan            = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    double error_prior_aniso_lapl = 0.0;
    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];
        const double area = area_prop[ inner_edges_prop[0][i] ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;


        // compute laplace(current edge)
        Point lapl_xe(0, 0, 0);
        lapl_xe += wq * template_points[vq];
        lapl_xe += ws * template_points[vs];
        lapl_xe += wr * template_points[vr];
        lapl_xe += wp * template_points[vp];

        // get laplace(xi)
        const Point& lapl_oldedge = template_aniso_laplace[ inner_edges_prop[0][i] ];

        if (templ_local_weight)
        {
            local_weight = (templ_local_weight[vp] + templ_local_weight[vq]) * Scalar(0.5);
        }

        double u = norm( lapl_xe - lapl_oldedge );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * u * u;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * std::pow( u, robustness_parameters_.parameter );
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( huber_thr*u - huber_thr*huber_thr/2.0 );
            }
            else
            {
                error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( u*u/2.0 );
            }
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            double scale_parameter = robustness_parameters_.parameter;
//                error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
            error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3
        }
        else // default to L2
        {
            error_prior_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * u * u;
        }
    }
    error_prior_aniso_lapl *= (weight_ / surface_area_aniso);

    return error_prior_aniso_lapl;
}


//-----------------------------------------------------------------------------

void
Energy_term_aniso_laplace::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_aniso_laplace::add_rows_small_lgs(...)'" << std::endl;
#endif

    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
    auto area_prop              = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan                  = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop       = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    auto templ_local_weight     = template_mesh->get_vertex_property<double>("v:regularization_weight");
    double local_weight = 1.0;

    const double weight_aniso_laplace = sqrt( weight_ / energy_term_data_.surface_area_aniso );
    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        const double area = area_prop[ inner_edges_prop[0][i] ];

        if (templ_local_weight)
        {
            local_weight = (templ_local_weight[vp] + templ_local_weight[vq]) * Scalar(0.5);
            local_weight = sqrt(local_weight);
        }

        // right hand side
        for ( unsigned int j = 0; j < 3; ++j )
        {
            B(r, j) = weight_aniso_laplace * local_weight * sqrt(area) * template_aniso_laplace[ inner_edges_prop[0][i] ][j];
        }

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // scale to weight smoothing vs. fitting. use sqrt of weight
        // since it will be squared for the energy. energy should be
        // sum_i (area_i * norm(laplace_i)), so weight is sqrt(area_i)
        wq  *= weight_aniso_laplace * local_weight * sqrt(area);
        ws  *= weight_aniso_laplace * local_weight * sqrt(area);
        wr  *= weight_aniso_laplace * local_weight * sqrt(area);
        wp  *= weight_aniso_laplace * local_weight * sqrt(area);

        // add coefficients to matrix
        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vq.idx(), wq) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vs.idx(), ws) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vr.idx(), wr) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vp.idx(), wp) );

        ++r;
    }

//    std::cerr << "out: 'Energy_term_aniso_laplace::add_rows_small_lgs(...)'" << std::endl;
}


//-----------------------------------------------------------------------------

void
Energy_term_aniso_laplace::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_aniso_laplace::add_rows_big_lgs(...)'" << std::endl;
#endif

    // TODO currently not supported
    std::cerr << "Energy_term_aniso_laplace::add_rows_big_lse(...) currently not supported" << std::endl;
    exit(1);
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_aniso_laplace::
n_rows_small_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    auto inner_edges_prop = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    return (unsigned int) inner_edges_prop[0].size();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
