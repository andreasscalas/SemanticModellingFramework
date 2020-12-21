//== INCLUDES =================================================================

#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/Energy_term_list.h>

#include <graphene/surface_mesh/algorithms/templatefitting/energy_terms/energy_terms.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================

Energy_term_list::Energy_term_list(const Energy_term_data& energy_term_data) :
    available_terms_(ET_NUM_ENERGY_TERMS, NULL),
    idx_of_termination_term_(0)
{
    available_terms_[ET_ANISO_LAPLACE] =                new Energy_term_aniso_laplace(energy_term_data, 0.0);
//    available_terms_[ET_ARAP] =                         new Energy_term_arap(energy_term_data, 0.0);
    available_terms_[ET_EYES_CONTOUR] =                 new Energy_term_eyes_contour(energy_term_data, 0.0);
    available_terms_[ET_KEEP_VERTICES] =                new Energy_term_keep_vertices(energy_term_data, 0.0);
    available_terms_[ET_LANDMARKS] =                    new Energy_term_landmarks(energy_term_data, 0.0);
    available_terms_[ET_EARS_LM] =                      new Energy_term_ears(energy_term_data, 0.0);
    available_terms_[ET_MOUTH_SHUT] =                    new Energy_term_mouth_shut(energy_term_data, 0.0);
    available_terms_[ET_EYES_CLOSED] =                  new Energy_term_eyes_closed(energy_term_data, 0.0);
//    available_terms_[ET_LAPLACE_ZERO] =                 new Energy_term_laplace_zero(energy_term_data, 0.0);
    available_terms_[ET_LAPLACIAN_COORDINATES] =        new Energy_term_laplacian_coordinates(energy_term_data, 0.0);
    available_terms_[ET_NONLIN_ANISO_LAPLACE] =         new Energy_term_nonlin_aniso_laplace(energy_term_data, 0.0);
    available_terms_[ET_NONLIN_LAPLACIAN_COORDINATES] = new Energy_term_nonlin_laplacian_coordinates(energy_term_data, 0.0);
//    available_terms_[ET_ORIGINAL_STRUCTURE] =           new Energy_term_original_structure(energy_term_data, 0.0);
    available_terms_[ET_PCA_CPC_MESH2PS] =              new Energy_term_pca_cpc_mesh2ps(energy_term_data, 0.0);
    available_terms_[ET_PCA_CPC_PS2MESH] =              new Energy_term_pca_cpc_ps2mesh(energy_term_data, 0.0);
//    available_terms_[ET_PCA_DYN_2D3D_REG] =             new Energy_term_pca_dyn2d3dreg(energy_term_data, 0.0);
    available_terms_[ET_PCA_LANDMARKS] =                new Energy_term_pca_landmarks(energy_term_data, 0.0);
    available_terms_[ET_POINT_TO_PLANE] =               new Energy_term_point_to_plane(energy_term_data, 0.0);
    available_terms_[ET_POINT_TO_POINT] =               new Energy_term_point_to_point(energy_term_data, 0.0);
    available_terms_[ET_PROJECTION_LEFT_EYE] =          new Energy_term_projection_left_eye(energy_term_data, 0.0);
    available_terms_[ET_PROJECTION_RIGHT_EYE] =         new Energy_term_projection_right_eye(energy_term_data, 0.0);
    available_terms_[ET_PROJECTION_Y_ZERO] =            new Energy_term_projection_y_zero(energy_term_data, 0.0);
//    available_terms_[ET_RIGID_LINEARIZED] =             new Energy_term_rigid_linearized(energy_term_data, 0.0);
//    available_terms_[ET_TIKHONOV_ARAP_LINEARIZED] =     new Energy_term_tikhonov_arap_linearized(energy_term_data, 0.0);
    available_terms_[ET_TIKHONOV_PCA] =                 new Energy_term_tikhonov_pca(energy_term_data, 0.0);
//    available_terms_[ET_TIKHONOV_RIGID_LINEARIZED] =    new Energy_term_tikhonov_rigid_linearized(energy_term_data, 0.0);
}

Energy_term_list::~Energy_term_list()
{
    size_t i;
    for (i=0; i < available_terms_.size(); ++i)
    {
        if (available_terms_[i] != NULL)
            delete available_terms_[i];
    }
}

Energy_term* Energy_term_list::get_term(size_t idx) const
{
    return available_terms_[idx];
}

void Energy_term_list::update_and_init_active()
{
    update_active();

    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->init();
    }
}

void Energy_term_list::update_active()
{
    size_t i;
    Energy_term* term;

    active_terms_.clear();

    for (i=0; i < available_terms_.size(); ++i)
    {
        term = available_terms_[i];
        if (term->get_weight() > 0.0)
        {
            active_terms_.push_back(term);
        }
    }
}

void Energy_term_list::apply_weight_multipliers()
{
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->apply_multiply_factor();
    }
}

void Energy_term_list::apply_pre_minimize_action()
{
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->pre_minimize_action();
    }
}

void Energy_term_list::assemble_small_lse(std::vector<Tripl>& coeffs,
                                          Eigen::MatrixXd& B,
                                          unsigned int& r)
{
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->set_row_start(r);
        term->pre_sle_assemble_action();
        term->add_rows_small_lse(coeffs, B, r);
        term->set_row_end(r);
    }
}

void Energy_term_list::assemble_big_lse(std::vector<Tripl> &coeffs,
                                        Eigen::VectorXd &b,
                                        unsigned int &r)
{
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->set_row_start(r);
        term->pre_sle_assemble_action();
        term->add_rows_big_lse(coeffs, b, r);
        term->set_row_end(r);
    }
}


void Energy_term_list::apply_post_processing()
{
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->post_processing();
    }
}


double Energy_term_list::evaluate()
{
    double result = 0.0;
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        result += term->evaluate();
    }

    return result;
}


double Energy_term_list::evaluate_fitting()
{
    double result = 0.0;
    size_t i;
    Energy_term* term;
    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];

        if ( "Point to Point" == term->get_name() )
        {
            result += term->evaluate();
            break;
        }
    }

    return result;
}


unsigned int Energy_term_list::n_rows_big_sle()
{
    size_t i;
    unsigned int n_rows=0;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        n_rows += term->n_rows_big_sle();
    }
    return n_rows;
}

unsigned int Energy_term_list::n_rows_small_sle()
{
    size_t i;
    unsigned int n_rows=0;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        n_rows += term->n_rows_small_sle();
    }
    return n_rows;
}

void Energy_term_list::set_term_for_termination_lambda(const std::string &name)
{
    size_t i;
    Energy_term* term;

    for (i=0; i < available_terms_.size(); ++i)
    {
        term = available_terms_[i];

        if (term->get_name() == name)
        {
            idx_of_termination_term_ = (unsigned int)i;
            break;
        }
    }
}

void Energy_term_list::set_term_for_termination_lambda(unsigned int term_idx)
{
    idx_of_termination_term_ = term_idx;
}


void Energy_term_list::clear()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < available_terms_.size(); ++i)
    {
        term = available_terms_[i];
        term->reset();
    }

    active_terms_.clear();
}


bool Energy_term_list::is_small_sle_supported()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        if (!term->small_sle_supported())
            return false;
    }

    return true;
}

bool Energy_term_list::need_correspondences()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        if (term->needs_correspondences())
            return true;
    }

    return false;
}

bool Energy_term_list::need_pca()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        if (term->needs_pca())
            return true;
    }

    return false;
}

bool Energy_term_list::is_non_linear()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        if (term->is_non_linear())
            return true;
    }

    return false;
}

bool Energy_term_list::is_all_L2()
{
    size_t i;
    Energy_term* term;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];

        if (term->get_robustness_params().solver_type != SOLVER_TYPE_L2)
            return false;
    }

    return true;
}


const std::set<Solve_result> Energy_term_list::solve_results()
{
    size_t i;
    Energy_term* term;
    std::set<Solve_result> sr;

    for (i=0; i < active_terms_.size(); ++i)
    {
        term = active_terms_[i];
        term->solve_result(sr);
    }

    return sr;
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
