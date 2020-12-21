//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_TIKHONOV_PCA_H
#define GRAPHENE_ENERGY_TERM_TIKHONOV_PCA_H


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
Energy_term_tikhonov_pca : public Energy_term
{
public:

    Energy_term_tikhonov_pca(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_tikhonov_pca();

    virtual double evaluate();

    virtual void add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r );

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return false; }

    virtual bool needs_pca() {return true;}


    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_PCA);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_TIKHONOV_PCA_H
//=============================================================================
