//== INCLUDES =================================================================


#include "Energy_term_original_structure.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_original_structure::
Energy_term_original_structure(const Energy_term_data& energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Original Structure")
{}


//-----------------------------------------------------------------------------


Energy_term_original_structure::
~Energy_term_original_structure()
{}


//-----------------------------------------------------------------------------


double
Energy_term_original_structure::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;


    const unsigned int nv = template_mesh->n_vertices();

    auto template_points      = template_mesh->get_vertex_property<Point>("v:point");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");

    double error_prior_local_struct = 0.0;
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            const Point& xi = template_restpose_points[v];
            const Point& xj = template_restpose_points[template_mesh->to_vertex(one_ring[or_it])];
            const Point& zi = template_points[v];
            const Point& zj = template_points[template_mesh->to_vertex(one_ring[or_it])];

            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            double weight_local_vertex = cotan[curr_edg] / 2.0;

            double u = norm( (zj - zi) - (xj - xi) );
            if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
            {
                error_prior_local_struct += weight_local_vertex * u * u;
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
            {
                error_prior_local_struct += weight_local_vertex * std::pow( u, robustness_parameters_.parameter );
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
            {
                double huber_thr = robustness_parameters_.parameter;
                if ( u > huber_thr )
                {
                    error_prior_local_struct += weight_local_vertex * ( huber_thr*u - huber_thr*huber_thr/2.0 );
                }
                else
                {
                    error_prior_local_struct += weight_local_vertex * ( u*u/2.0 );
                }
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
            {
                double scale_parameter = robustness_parameters_.parameter;
//                    error_prior_local_struct += weight_local_vertex * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
                error_prior_local_struct += weight_local_vertex * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3
            }
            else // default to L2
            {
                error_prior_local_struct += weight_local_vertex * u * u;
            }
        }
    }
    error_prior_local_struct *= (weight_ / nv);

    return error_prior_local_struct;
}




//-----------------------------------------------------------------------------

void
Energy_term_original_structure::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_original_structure::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
#ifdef BE_VERBOSE
    std::cerr << "in: 'add_Energy_prior_original_local_structure(...)'" << std::endl;
#endif

    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto cotan = template_mesh->get_edge_property<double>("e:cotan");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_prior_local_struct = sqrt(weight_ / nv);
    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            const Point& sn = template_restpose_points[template_mesh->to_vertex(one_ring[or_it])];

            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            double weight_local_vertex = cotan[curr_edg] / 2.0;
            // std::cerr << "[DEBUG] weight_local_vertex: " << weight_local_vertex << std::endl;
            if (weight_local_vertex < 0.0) // TODO
            {
                weight_local_vertex = 0;
            }
            else
            {
                weight_local_vertex = sqrt(weight_local_vertex);
            }

            // right hand side
            for (unsigned int j = 0; j < 3; ++j)
            {
                B(r, j) = weight_prior_local_struct * weight_local_vertex * ( sn[j] - template_restpose_points[v][j] );
            }

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match +                     i,  -weight_prior_local_struct * weight_local_vertex) ); // x_i (bzw. z_i)
            coeffs.push_back( Tripl(r, column_offsets.offset_match + template_mesh->to_vertex(one_ring[or_it]).idx(), weight_prior_local_struct * weight_local_vertex) ); // x_j's
            ++r;
        }

        ++i;
    }
}


//-----------------------------------------------------------------------------

void
Energy_term_original_structure::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_original_structure::add_rows_big_lgs(...)'" << std::endl;
#endif

    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_prior_local_struct = sqrt(weight_ / nv);
    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            const Point& sn = template_restpose_points[template_mesh->to_vertex(one_ring[or_it])];

            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            double weight_local_vertex = cotan[curr_edg] / 2.0;
            weight_local_vertex = sqrt(weight_local_vertex);

            // right hand side
            b(r) = weight_prior_local_struct * weight_local_vertex * ( sn[0] - template_restpose_points[v][0] );

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i                                               , -weight_prior_local_struct * weight_local_vertex) ); // x_i (bzw. z_i)
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(one_ring[or_it]).idx(),  weight_prior_local_struct * weight_local_vertex) ); // x_j's
            ++r;

            // right hand side
            b(r) = weight_prior_local_struct * weight_local_vertex * ( sn[1] - template_restpose_points[v][1] );

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i + 1                                               , -weight_prior_local_struct * weight_local_vertex) ); // x_i (bzw. z_i)
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 1,  weight_prior_local_struct * weight_local_vertex) ); // x_j's
            ++r;

            // right hand side
            b(r) = weight_prior_local_struct * weight_local_vertex * ( sn[2] - template_restpose_points[v][2] );

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i + 2                                               , -weight_prior_local_struct * weight_local_vertex) ); // x_i (bzw. z_i)
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 2,  weight_prior_local_struct * weight_local_vertex) ); // x_j's
            ++r;
        }

        ++i;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_original_structure::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_original_structure::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    unsigned int n = 0;
    for (auto v : template_mesh->vertices())
    {
        for (auto hv : template_mesh->halfedges(v))
        {
            ++n;
        }
    }
    return n;
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_original_structure::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_original_structure::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    unsigned int n = 0;
    for (auto v : template_mesh->vertices())
    {
        for (auto hv : template_mesh->halfedges(v))
        {
            n += 3;
        }
    }
    return n;
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
