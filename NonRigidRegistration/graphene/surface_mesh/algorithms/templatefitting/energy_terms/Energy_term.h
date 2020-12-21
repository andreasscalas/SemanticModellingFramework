//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_H
#define GRAPHENE_ENERGY_TERM_H


//== INCLUDES =================================================================


#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term_data.h>
#include <graphene/geometry/Point_set.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <set>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


enum Available_energy_terms
{
    ET_ANISO_LAPLACE = 0,
//    ET_ARAP,
    ET_EYES_CONTOUR,
    ET_KEEP_VERTICES,
    ET_LANDMARKS,
    ET_EARS_LM,
    ET_MOUTH_SHUT,
    ET_EYES_CLOSED,
//    ET_LAPLACE_ZERO,
    ET_LAPLACIAN_COORDINATES,
    ET_NONLIN_ANISO_LAPLACE,
    ET_NONLIN_LAPLACIAN_COORDINATES,
//    ET_ORIGINAL_STRUCTURE,
    ET_PCA_CPC_MESH2PS,
    ET_PCA_CPC_PS2MESH,
//    ET_PCA_DYN_2D3D_REG,
    ET_PCA_LANDMARKS,
    ET_POINT_TO_PLANE,
    ET_POINT_TO_POINT,
    ET_PROJECTION_LEFT_EYE,
    ET_PROJECTION_RIGHT_EYE,
    ET_PROJECTION_Y_ZERO,
//    ET_RIGID_LINEARIZED,
//    ET_TIKHONOV_ARAP_LINEARIZED,
    ET_TIKHONOV_PCA,
//    ET_TIKHONOV_RIGID_LINEARIZED,
    ET_NUM_ENERGY_TERMS
};

enum Solve_result
{
    SR_GLOBAL_ROT_TRANS = 0,
    SR_MATRIX_PER_VERTEX,
    SR_VERTICES,
    SR_PCA,
    SR_NONE
};

enum Solver_type {
    SOLVER_TYPE_L2,
    SOLVER_TYPE_L1,
    SOLVER_TYPE_HUBER,
    SOLVER_TYPE_GEMANMCCLURE,
    SOLVER_TYPE_MAX
};


struct Robustness_parameters
{
    Solver_type solver_type;
    double parameter;

    Robustness_parameters() :
        solver_type(SOLVER_TYPE_L2),
        parameter(0.0)
    {}
};


class
Energy_term
{
public:

    Energy_term(const Energy_term_data& energy_term_data, double weight = 0.0, const std::string& name = "Energy_term");

    virtual ~Energy_term();

    virtual const std::string& get_name() const {return name_;}

    ///called right before fitting procedure
    virtual void init() {}
    /// called in third loop before minimize
    virtual void pre_minimize_action() {}
    /// called in third loop before assembling the linear system
    virtual void pre_sle_assemble_action() {}
    ///called in third loop after solve and application of solution
    virtual void post_processing() {}

    virtual double evaluate() {return 0.0;}

    virtual void add_rows_small_lse( std::vector<Tripl>& coeffs,
                                     Eigen::MatrixXd& B,
                                     unsigned int& r) {}

    virtual void add_rows_big_lse( std::vector<Tripl>& coeffs,
                                   Eigen::VectorXd& b,
                                   unsigned int& r) {}

    virtual unsigned int n_rows_small_sle() { return 0; }
    virtual unsigned int n_rows_big_sle() { return 0; }

    virtual void reset();

    virtual bool is_non_linear() {return false;}

    /// implement in subclass; return true iff small linear system is supported by energy term.
    virtual bool small_sle_supported() {return false;}

    /// implement in subclass; return true iff energy term needs correspondences
    virtual bool needs_correspondences() {return false;}

    /// implement in subclass;
    virtual bool needs_pca() {return false;}

    /// implement in subclass; add specific Solve_result to solve_results reference
    virtual void solve_result(std::set<Solve_result>& solve_results) {}

    /// set weight for energy term
    virtual void set_weight(double weight) { weight_ = weight;}
    /// return current weight of energy term
    virtual double get_weight() { return weight_; }

    /// set multiply factor for decrease of increase of weight in each iteration
    virtual void set_multiply_factor(double factor) {multiply_factor_ = factor;}
    /// return multiply factor
    virtual double get_multiply_factor() const { return multiply_factor_; }
    /// apply multiply factor to weight
    virtual void apply_multiply_factor() { weight_ *= multiply_factor_; }

    virtual void set_robustness_params(const Robustness_parameters& params) {robustness_parameters_ = params;}
    virtual const Robustness_parameters& get_robustness_params() {return robustness_parameters_;}

    virtual void set_row_start(unsigned int r) { row_start_ = r; }
    virtual unsigned int get_row_start() { return row_start_; }
    virtual void set_row_end(unsigned int r) { row_end_ = r; }
    virtual unsigned int get_row_end() { return row_end_; }


protected:

    const Energy_term_data& energy_term_data_;

    std::string             name_;

    double                  weight_;

    double                  multiply_factor_;

    unsigned int            row_start_,row_end_;

    Robustness_parameters   robustness_parameters_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_H
//=============================================================================
