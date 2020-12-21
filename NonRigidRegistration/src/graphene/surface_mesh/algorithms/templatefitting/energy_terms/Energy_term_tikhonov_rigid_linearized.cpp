//== INCLUDES =================================================================


#include "Energy_term_tikhonov_rigid_linearized.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_tikhonov_rigid_linearized::
Energy_term_tikhonov_rigid_linearized(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Tikhonov Rigid Linearized")
{
}


//-----------------------------------------------------------------------------


Energy_term_tikhonov_rigid_linearized::
~Energy_term_tikhonov_rigid_linearized()
{}


//-----------------------------------------------------------------------------


double
Energy_term_tikhonov_rigid_linearized::
evaluate()
{
    // TODO currently not supported
    std::cerr << "Energy_term_tikhonov_rigid_linearized::evaluate(...) currently not supported" << std::endl;
    exit(1);

    // weights_settings.weight_reg_rigid_regularize;
}


//-----------------------------------------------------------------------------


void
Energy_term_tikhonov_rigid_linearized::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r )
{

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    // TODO currently not supported
    std::cerr << "Energy_term_tikhonov_rigid_linearized::add_rows_big_lse(...) currently not supported" << std::endl;
    exit(1);

    
    const unsigned int offset_nrd_w_rigid = column_offsets.offset_nrd_w_rigid;


    
#ifdef BE_VERBOSE
    std::cerr << "in: 'add_Energy_regularize_rigid_3er(...)'" << std::endl;
#endif

    const double weight_reg = sqrt(weight_ / 6.0);
    for (unsigned int i = 0; i < 6; ++i)
    {
        // right hand side
        b(r) = 0.0;

        // begin_row
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + i, weight_reg ) );
        ++r;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_tikhonov_rigid_linearized::
n_rows_big_sle()
{
    return 6;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
