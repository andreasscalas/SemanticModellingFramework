//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_PCA_DYN2D3DREG_H
#define GRAPHENE_ENERGY_TERM_PCA_DYN2D3DREG_H


//== INCLUDES =================================================================


#include "Energy_term.h"

#include <vector>

#include "../my_types.h"
#include <graphene/surface_mesh/algorithms/pca/PCA.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_pca_dyn2d3dreg : public Energy_term
{
public:

    Energy_term_pca_dyn2d3dreg(const Energy_term_data& energy_term_data, double weight);

    ~Energy_term_pca_dyn2d3dreg();

    double evaluate();


    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return false; }

    virtual bool needs_pca() {return true;}

    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_VERTICES); solve_results.insert(SR_PCA);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_PCA_DYN2D3DREG_H
//=============================================================================
