//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_PROJECTION_LEFT_EYE_H
#define GRAPHENE_ENERGY_TERM_PROJECTION_LEFT_EYE_H


//== INCLUDES =================================================================


#include "Energy_term.h"

#include <vector>

#include "../my_types.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_projection_left_eye : public Energy_term
{
public:

    Energy_term_projection_left_eye(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_projection_left_eye();

    virtual void pre_minimize_action();

    virtual double evaluate();

    virtual void add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r );

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_small_sle();
    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return true; }

    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_VERTICES);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_PROJECTION_LEFT_EYE_H
//=============================================================================
