//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_PCA_CPC_PS2MESH_H
#define GRAPHENE_ENERGY_TERM_PCA_CPC_PS2MESH_H


//== INCLUDES =================================================================


#include "Energy_term.h"

#include "../my_types.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_pca_cpc_ps2mesh : public Energy_term
{
private:

    std::vector<Correspondence> skinned_correspondences_;

public:

    Energy_term_pca_cpc_ps2mesh(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_pca_cpc_ps2mesh();

    virtual void pre_minimize_action();

    virtual double evaluate();

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r );

    virtual unsigned int n_rows_big_sle();

    virtual bool small_sle_supported() { return false; }

    virtual bool needs_correspondences() { return true; }

    virtual bool needs_pca() {return true;}

    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_PCA);}

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_PCA_CPC_PS2MESH_H
//=============================================================================
