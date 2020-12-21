//== INCLUDES =================================================================


#include "Energy_term_laplace_zero.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_laplace_zero::
Energy_term_laplace_zero(const Energy_term_data& energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Laplace Zero")
{}


//-----------------------------------------------------------------------------


Energy_term_laplace_zero::
~Energy_term_laplace_zero()
{}


//-----------------------------------------------------------------------------

double
Energy_term_laplace_zero::
evaluate()
{
    double surface_area = energy_term_data_.surface_area;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");
    auto area            = template_mesh->get_vertex_property<double>("v:area");
    auto cotan           = template_mesh->get_edge_property<double>("e:cotan");

    double error_prior_lapl_z = 0.0;
    for (auto v : template_mesh->vertices())
    {
        // compute laplace(zi)
        Point lapl_zi(0, 0, 0);
        for (auto hc : template_mesh->halfedges(v))
        {
            const double w = cotan[template_mesh->edge(hc)] / (2*area[v]);
            lapl_zi       += w * (template_points[template_mesh->to_vertex(hc)] - template_points[v]);
        }

        error_prior_lapl_z += area[v] * sqrnorm( lapl_zi );
    }
    error_prior_lapl_z *= (weight_ / surface_area);

    return error_prior_lapl_z;
}


//-----------------------------------------------------------------------------

void
Energy_term_laplace_zero::
add_rows_small_lse(
        std::vector<Tripl> &coeffs,
        Eigen::MatrixXd &B,
        unsigned int &r)
{

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_laplace_zero::add_rows_small_lgs(...)'" << std::endl;
#endif

    auto area  = template_mesh->get_vertex_property<double>("v:area");
    auto cotan = template_mesh->get_edge_property<double>("e:cotan");

    const double weight_prior_laplace_z = sqrt(weight_ / energy_term_data_.surface_area);
    const unsigned int nv = template_mesh->n_vertices();
    for (unsigned int i = 0; i < nv; ++i)
    {
        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = 0.0;
        }

        Surface_mesh::Vertex v(i);

        double ww(0), w;
        // begin_row
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_z * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + template_mesh->to_vertex(hc).idx(), w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + i, -ww) );
        ++r;
    }
}



//-----------------------------------------------------------------------------

void
Energy_term_laplace_zero::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{

    const Column_offsets &column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;


#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_laplace_zero::add_rows_big_lgs(...)'" << std::endl;
#endif

    auto area  = template_mesh->get_vertex_property<double>("v:area");
    auto cotan = template_mesh->get_edge_property<double>("e:cotan");

    const double weight_prior_laplace_z = sqrt(weight_ / energy_term_data_.surface_area);
    unsigned int i = 0;
    for (auto v: template_mesh->vertices())
    {
        // right hand side
        b(r) = 0.0;

        double ww(0), w;
        // begin_row
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_z * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(hc).idx(), w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i, -ww) );
        ++r;

        // right hand side
        b(r) = 0.0;

        ww = 0;
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_z * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
            ww += w;
            coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*template_mesh->to_vertex(hc).idx() + 1, w) );
        }
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*i + 1, -ww) );
        ++r;

        // right hand side
        b(r) = 0.0;

        ww = 0;
        for (auto hc : template_mesh->halfedges(v))
        {
            w   = weight_prior_laplace_z * cotan[template_mesh->edge(hc)] / (2*sqrt(area[v]));
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
Energy_term_laplace_zero::
n_rows_small_sle()
{
    return energy_term_data_.template_mesh->n_vertices();
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_laplace_zero::
n_rows_big_sle()
{
    return 3*energy_term_data_.template_mesh->n_vertices();
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
