//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_TIKHONOV_RIGID_LINEARIZED_H
#define GRAPHENE_ENERGY_TERM_TIKHONOV_RIGID_LINEARIZED_H


//== INCLUDES =================================================================


#include "Energy_term.h"


#include <vector>

#include "../my_types.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_tikhonov_rigid_linearized : public Energy_term
{
public:

    Energy_term_tikhonov_rigid_linearized(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_tikhonov_rigid_linearized();

    virtual double evaluate();



    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return false; }


    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_GLOBAL_ROT_TRANS);}
};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_TIKHONOV_RIGID_LINEARIZED_H
//=============================================================================
