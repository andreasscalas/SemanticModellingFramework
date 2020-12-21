//== INCLUDES =================================================================


#include "Energy_term_laplacian_coordinates.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_laplacian_coordinates::
Energy_term_laplacian_coordinates(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Laplacian Coordinates")
{
}


//-----------------------------------------------------------------------------


Energy_term_laplacian_coordinates::
~Energy_term_laplacian_coordinates()
{}


//-----------------------------------------------------------------------------


double
Energy_term_laplacian_coordinates::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points                = template_mesh->get_vertex_property<Point>("v:point");
    auto template_laplacian_coordinates = template_mesh->get_vertex_property<Point>("v:laplacian_coordinates");
    auto area                           = template_mesh->get_vertex_property<double>("v:area");
    auto cotan                          = template_mesh->get_edge_property<double>("e:cotan");

    double error_prior_lapl_d = 0.0;
    for (auto v : template_mesh->vertices())
    {
        // compute laplace(zi)
        Point lapl_zi(0, 0, 0);
        for (auto hc : template_mesh->halfedges(v))
        {
            const double w = cotan[template_mesh->edge(hc)] / (2*area[v]);
            lapl_zi       += w * (template_points[template_mesh->to_vertex(hc)] - template_points[v]);
        }

        // get laplace(xi)
        const Point& lapl_xi = template_laplacian_coordinates[v];

        double u = norm( lapl_zi - lapl_xi );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_prior_lapl_d += area[v] * u * u;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_prior_lapl_d += area[v] * std::pow( u, robustness_parameters_.parameter );
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_prior_lapl_d += area[v] * ( huber_thr*u - huber_thr*huber_thr/2.0 );
            }
            else
            {
                error_prior_lapl_d += area[v] * ( u*u/2.0 );
            }
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            double scale_parameter = robustness_parameters_.parameter;
//                error_prior_lapl_d += area[v] * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
            error_prior_lapl_d += area[v] * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3
        }
        else // default to L2
        {
            error_prior_lapl_d += area[v] * u * u;
        }
    }
    error_prior_lapl_d *= (weight_ / energy_term_data_.surface_area);

    return error_prior_lapl_d;
}


//-----------------------------------------------------------------------------


void
Energy_term_laplacian_coordinates::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_laplacian_coordinates::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;


    auto template_laplacian_coordinates = template_mesh->get_vertex_property<Point>("v:laplacian_coordinates");
    auto area                           = template_mesh->get_vertex_property<double>("v:area");
    auto cotan                          = template_mesh->get_edge_property<double>("e:cotan");

    const double weight_prior_laplace_d = sqrt(weight_ / energy_term_data_.surface_area);
    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = weight_prior_laplace_d * sqrt(area[v]) * template_laplacian_coordinates[v][j];
        }

        double ww(0), w;
        // begin_row
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_d * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + template_mesh->to_vertex(hc).idx(), w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + i, -ww) );
        ++r;

        ++i;
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_laplacian_coordinates::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_laplacian_coordinates::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_laplacian_coordinates = template_mesh->get_vertex_property<Point>("v:laplacian_coordinates");
    auto area                           = template_mesh->get_vertex_property<double>("v:area");
    auto cotan                          = template_mesh->get_edge_property<double>("e:cotan");

    const double weight_prior_laplace_d = sqrt(weight_ / energy_term_data_.surface_area);
    unsigned int i = 0;
    for (auto v: template_mesh->vertices())
    {
        // right hand side
        b(r) = weight_prior_laplace_d * sqrt(area[v]) * template_laplacian_coordinates[v][0];

        double ww(0), w;
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_d * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(hc).idx(), w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i, -ww) );
        ++r;

        // right hand side
        b(r) = weight_prior_laplace_d * sqrt(area[v]) * template_laplacian_coordinates[v][1];

        ww = 0;
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_d * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(hc).idx() + 1, w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i + 1, -ww) );
        ++r;

        // right hand side
        b(r) = weight_prior_laplace_d * sqrt(area[v]) * template_laplacian_coordinates[v][2];

        ww = 0;
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_d * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(hc).idx() + 2, w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i + 2, -ww) );
        ++r;

        ++i;
    }
}



//-----------------------------------------------------------------------------


unsigned int
Energy_term_laplacian_coordinates::
n_rows_small_sle()
{
    return energy_term_data_.template_mesh->n_vertices();
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_laplacian_coordinates::
n_rows_big_sle()
{
    return 3*energy_term_data_.template_mesh->n_vertices();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
