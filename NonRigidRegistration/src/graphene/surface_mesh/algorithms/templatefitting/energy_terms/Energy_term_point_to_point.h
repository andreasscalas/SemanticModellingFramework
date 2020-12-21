//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_POINT_TO_POINT_H
#define GRAPHENE_ENERGY_TERM_POINT_TO_POINT_H


//== INCLUDES =================================================================


#include "Energy_term.h"

#include <vector>

#include "../my_types.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_point_to_point : public Energy_term
{
private:
    std::vector<Correspondence> skinned_correspondences_;
public:

    Energy_term_point_to_point(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_point_to_point();

    virtual void pre_minimize_action();

    virtual double evaluate();

    virtual void add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r);

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r);

    virtual unsigned int n_rows_small_sle();
    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return true; }

    virtual bool needs_correspondences() { return true; }


    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_VERTICES);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_POINT_TO_POINT_H
//=============================================================================
