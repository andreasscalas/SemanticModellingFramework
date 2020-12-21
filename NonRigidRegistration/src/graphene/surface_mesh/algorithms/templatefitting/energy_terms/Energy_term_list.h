//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_LIST_H
#define GRAPHENE_ENERGY_TERM_LIST_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================




class
Energy_term_list
{
public:

    Energy_term_list(const Energy_term_data &energy_term_data);

    ~Energy_term_list();

    /// get size of FULL list
    size_t size() const { return available_terms_.size(); }

    /// get a specific term; best use Available_energy_terms enum
    Energy_term* get_term(size_t idx) const;


    /// initialize active energy terms
    void update_and_init_active();

    /// checks all available terms if their weight is greater 0.0 and puts them in the active list
    void update_active();


    /// apply weight multipliers to all weights of the active terms
    void apply_weight_multipliers();

    /// apply pre-minimize action to all active terms
    void apply_pre_minimize_action();


    /// assemble small linear system from active terms
    void assemble_small_lse(std::vector<Tripl>& coeffs,
                            Eigen::MatrixXd& B,
                            unsigned int& r);

    /// assemble big linear system from active terms
    void assemble_big_lse(std::vector<Tripl>& coeffs,
                          Eigen::VectorXd& b,
                          unsigned int& r);

    /// applied after solve and application of solution
    void apply_post_processing();

    double evaluate();
    double evaluate_fitting();

    /// determines the number of rows for the big sle
    unsigned int n_rows_big_sle();
    /// determines the number of rows for the small sle
    unsigned int n_rows_small_sle();


    /// find index of the term which lambda shall be used as termination criteria
    void set_term_for_termination_lambda(const std::string& name);
    /// set index of the term which lambda shall be used as termination criteria
    void set_term_for_termination_lambda(unsigned int term_idx);
    Energy_term* get_term_for_termination_lambda() {return available_terms_[idx_of_termination_term_];}
    /// get the weight of term for termination lambda
    double get_termination_lambda() { return get_term(idx_of_termination_term_)->get_weight(); }


    /// clears the active term list and resets all energies
    void clear();

    ///true iff small sle is supported by all active terms
    bool is_small_sle_supported();
    ///true iff one or more active terms need correspondences
    bool need_correspondences();
    ///true iff one or more active terms need pca
    bool need_pca();
    ///true iff one or more active terms are non-linear
    bool is_non_linear();
    ///true iff all solver types are L2
    bool is_all_L2();

    ///the solve results of the active terms
    const std::set<Solve_result> solve_results();

    const std::vector<Energy_term*> active_terms() { return active_terms_; }

private:
    std::vector<Energy_term*> available_terms_;

    std::vector<Energy_term*> active_terms_;

    unsigned int              idx_of_termination_term_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_LIST_H
//=============================================================================
