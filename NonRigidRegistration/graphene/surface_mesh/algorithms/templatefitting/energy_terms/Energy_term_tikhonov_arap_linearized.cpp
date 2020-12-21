//== INCLUDES =================================================================


#include "Energy_term_tikhonov_arap_linearized.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_tikhonov_arap_linearized::
Energy_term_tikhonov_arap_linearized(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Tikhonov ARAP Linearized")
{
}


//-----------------------------------------------------------------------------


Energy_term_tikhonov_arap_linearized::
~Energy_term_tikhonov_arap_linearized()
{}


//-----------------------------------------------------------------------------


double
Energy_term_tikhonov_arap_linearized::
evaluate()
{
    // TODO currently not supported
    std::cerr << "Energy_term_tikhonov_arap_linearized::evaluate(...) currently not supported" << std::endl;
    exit(1);

    // weights_settings.weight_reg_arap_regularize;
}


//-----------------------------------------------------------------------------


void
Energy_term_tikhonov_arap_linearized::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r
                  
                   )
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    // TODO currently not supported
    std::cerr << "Energy_term_tikhonov_arap_linearized::add_rows_big_lse(...) currently not supported" << std::endl;
    exit(1);


    const unsigned int offset_nrd_w_arap = column_offsets.offset_nrd_w_arap;
    unsigned int nv = template_mesh->n_vertices();




#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_tikhonov_arap_linearized::add_rows_big_lgs(...)'" << std::endl;
#endif

    const double weight_reg = sqrt(weight_ / (3*nv));
    for (unsigned int i = 0; i < nv; ++i)
    {
        // right hand side
        b(r) = 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i, weight_reg ) ); // alpha_i
        ++r;

        // right hand side
        b(r) = 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 1, weight_reg ) ); // beta_i
        ++r;

        // right hand side
        b(r) = 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 2, weight_reg ) ); // gamma_i
        ++r;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_tikhonov_arap_linearized::
n_rows_big_sle()
{
    return 3*energy_term_data_.template_mesh->n_vertices();
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
