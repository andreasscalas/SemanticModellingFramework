//== INCLUDES =================================================================


#include "Energy_term.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term::
Energy_term(const Energy_term_data& energy_term_data, double weight, const std::string& name) :
    energy_term_data_(energy_term_data),
    name_(name),
    weight_(weight),
    multiply_factor_(1.0),
    row_start_(0),
    row_end_(0),
    robustness_parameters_()
{}


//-----------------------------------------------------------------------------


Energy_term::
~Energy_term()
{}


//-----------------------------------------------------------------------------


void
Energy_term::
reset()
{
    weight_ = 0.0;
    multiply_factor_ = 1.0;
    robustness_parameters_ = Robustness_parameters();
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
