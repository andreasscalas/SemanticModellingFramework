//=============================================================================


#ifndef GRAPHENE_ENERGY_H
#define GRAPHENE_ENERGY_H


//== INCLUDES =================================================================


#include <cfloat>
#include <vector>

#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term.h>
#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/energy_terms.h>
#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term_list.h>

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>

#include "my_types.h"
#include "Landmarks_manager.h"
#include "utility/LLS_solver.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {

using graphene::utility::LLS_solver;


//== CLASS DEFINITION =========================================================


class
Energy
{
public:

    Energy();

    ~Energy();

    void clear();

    bool minimize( const double  conv_val );

    double get_function_value();
    double evaluate_fitting();


    Energy_term_data& energy_term_data() { return energy_term_data_; }
    Energy_term_list& energy_term_list() { return energy_term_list_; }
    const Energy_term_list& energy_term_list() const { return energy_term_list_; }
private:


    unsigned int compute_offsets_and_return_number_of_columns(  const std::set<Solve_result>& solve_results, unsigned int nv, bool use_small_lgs, unsigned int pca_dim  );

    void solve();

    void apply_solution(const std::set<Solve_result>& solve_results, const Eigen::VectorXd& x);
    void apply_solution(const std::set<Solve_result>& solve_results, const Eigen::MatrixXd& X);


private:
    Energy_term_data energy_term_data_;

    Energy_term_list energy_term_list_;


private:

    double function_value_     = DBL_MAX;

    utility::LLS_solver      lls_solver_;

};

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_H
//=============================================================================
