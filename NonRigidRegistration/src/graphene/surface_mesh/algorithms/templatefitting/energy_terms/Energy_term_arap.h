//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_ARAP_H
#define GRAPHENE_ENERGY_TERM_ARAP_H


//== INCLUDES =================================================================


#include <vector>

#include "../my_types.h"
#include "Energy_term.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_arap : public Energy_term
{
public:

    Energy_term_arap(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_arap();

    virtual void pre_minimize_action();
    virtual void pre_sle_assemble_action();

    virtual double evaluate();

    virtual void add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r);

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r);

    virtual unsigned int n_rows_small_sle();
    virtual unsigned int n_rows_big_sle();

    virtual bool is_non_linear() {return true;}

    virtual bool small_sle_supported() { return true; }

    virtual void solve_result(std::set<Solve_result>& solve_results) {solve_results.insert(SR_VERTICES);}


//    double evaluate( Weights_settings& weights_settings,
//                     Robustness_settings& robustness_settings,
//                     Surface_mesh* template_mesh );

//    void add_rows_small_lgs_arap_alter( const double reg_weight, std::vector<Tripl>& coeffs, Eigen::MatrixXd& B, unsigned int& r const unsigned int offset_match, Surface_mesh* template_mesh );

//    void add_rows_big_lgs_arap_alter( const double reg_weight, std::vector<Tripl>& coeffs, Eigen::VectorXd& b, unsigned int& r const unsigned int offset_match, Surface_mesh* template_mesh );

//    void add_rows_big_lgs_arap_linearized( const double reg_weight, std::vector<Tripl>& coeffs, Eigen::VectorXd& b, unsigned int& r const unsigned int offset_match, const unsigned int offset_nrd_w_arap, Surface_mesh* template_mesh );

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_ARAP_H
//=============================================================================
