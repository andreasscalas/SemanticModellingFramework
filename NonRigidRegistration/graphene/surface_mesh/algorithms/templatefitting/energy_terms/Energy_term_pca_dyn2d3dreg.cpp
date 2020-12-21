//== INCLUDES =================================================================


#include "Energy_term_pca_dyn2d3dreg.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_pca_dyn2d3dreg::
Energy_term_pca_dyn2d3dreg(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "PCA Dynamic 2D-3D")
{
}


//-----------------------------------------------------------------------------


Energy_term_pca_dyn2d3dreg::
~Energy_term_pca_dyn2d3dreg()
{}


//-----------------------------------------------------------------------------


double
Energy_term_pca_dyn2d3dreg::
evaluate()
{
    // TODO currently not supported
    std::cerr << "Energy_term_pca_dyn2d3dreg::evaluate(...) currently not supported" << std::endl;
    exit(1);

    const PCA& pca = energy_term_data_.pca;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const unsigned int nv = template_mesh->n_vertices();

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_reg_pca = 0.0;

    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        const Point& zi = template_points[v];

        double mi_x = pca.m_pca_(3*i    );
        double mi_y = pca.m_pca_(3*i + 1);
        double mi_z = pca.m_pca_(3*i + 2);
        const Point  mi(mi_x, mi_y, mi_z); // TODO EVTL. SCHNELLER ?!

        Point  Pid(0.0, 0.0, 0.0);
        for (unsigned int j = 0; j < 3; ++j)
        {
            for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
            {
                Pid[j] += pca.P_pca_(3*i + j, dcount) * pca.d_pca_(dcount);
            }
        }

        error_reg_pca += sqrnorm( zi - Pid - mi );

        ++i;
    }
    error_reg_pca *= (weight_ / nv);

    return error_reg_pca;
}


//-----------------------------------------------------------------------------


void
Energy_term_pca_dyn2d3dreg::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    // TODO currently not supported
    std::cerr << "Energy_term_pca_dyn2d3dreg::evaluate(...) currently not supported" << std::endl;
    exit(1);

    
    const unsigned int offset_match = column_offsets.offset_match;
    const unsigned int offset_nrd_w_pca = column_offsets.offset_nrd_w_pca;
    const PCA& pca = energy_term_data_.pca;
    

#ifdef BE_VERBOSE
    std::cerr << "in: 'add_Energy_prior_PCA_3er(...)'" << std::endl;
#endif

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_model = sqrt(weight_ / nv);
    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        // right hand side
        b(r) = weight_model * pca.m_pca_(3*i + 0);

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i, weight_model) ); // z_i
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, -pca.P_pca_(3*i, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * pca.m_pca_(3*i + 1);

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i + 1, weight_model) ); // z_i
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, -pca.P_pca_(3*i + 1, dcount) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * pca.m_pca_(3*i + 2);

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i + 2, weight_model) ); // z_i
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, -pca.P_pca_(3*i + 2, dcount) * weight_model) ); // d's
        }
        ++r;

        ++i;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_pca_dyn2d3dreg::
n_rows_big_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    return 3*template_mesh->n_vertices();
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
