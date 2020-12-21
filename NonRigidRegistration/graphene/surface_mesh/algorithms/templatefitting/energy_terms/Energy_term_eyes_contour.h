//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_EYES_CONTOUR_H
#define GRAPHENE_ENERGY_TERM_EYES_CONTOUR_H


//== INCLUDES =================================================================


#include <vector>

#include "../my_types.h"
#include "Energy_term.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class
Energy_term_eyes_contour : public Energy_term
{
public:

    Energy_term_eyes_contour(const Energy_term_data& energy_term_data, double weight);

    virtual ~Energy_term_eyes_contour();


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

//    double evaluate( Weights_settings& weights_settings,
//                     Eyes_manager& eyes_manager,
//                     Surface_mesh* template_mesh );

//    void add_rows_small_lgs( const double match_weight,
//                             std::vector<Tripl>& coeffs,
//                             Eigen::MatrixXd& B,
//                             unsigned int& r
//                             const unsigned int offset_match,
//                             Eyes_manager& eyes_manager );

//    void add_rows_big_lgs( const double match_weight,
//                           std::vector<Tripl>& coeffs,
//                           Eigen::VectorXd& b,
//                           unsigned int& r
//                           const unsigned int offset_match,
//                           Eyes_manager& eyes_manager );

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_EYES_CONTOUR_H
//=============================================================================
