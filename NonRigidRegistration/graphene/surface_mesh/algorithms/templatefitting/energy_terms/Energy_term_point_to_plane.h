//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_POINT_TO_PLANE_H
#define GRAPHENE_ENERGY_TERM_POINT_TO_PLANE_H


//== INCLUDES =================================================================


#include "Energy_term.h"

#include <vector>

#include "../my_types.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_point_to_plane : public Energy_term
{
public:

    Energy_term_point_to_plane(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_point_to_plane();

    virtual double evaluate();


    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return false; }

    virtual bool needs_correspondences() { return true; }


    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_VERTICES);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_POINT_TO_PLANE_H
//=============================================================================
