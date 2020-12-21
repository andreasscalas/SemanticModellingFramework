//== INCLUDES =================================================================


#include "Energy_term_rigid_linearized.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_rigid_linearized::
Energy_term_rigid_linearized(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Rigid Linearized")
{
}


//-----------------------------------------------------------------------------


Energy_term_rigid_linearized::
~Energy_term_rigid_linearized()
{}


//-----------------------------------------------------------------------------


double
Energy_term_rigid_linearized::
evaluate()
{
    // TODO currently not supported
    std::cerr << "Energy_term_rigid_linearized::evaluate(...) currently not supported" << std::endl;
    exit(1);

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    graphene::Mat4f global_rigid_trans;  // TODO ALS PROPERTY !!!!!!

    
    const unsigned int nv = template_mesh->n_vertices();

    auto template_points                = template_mesh->get_vertex_property<Point>("v:point");
    auto template_restpose_points           = template_mesh->get_vertex_property<Point>("v:rest_pose");

    double error_reg_rigid = 0.0;
    for (auto v : template_mesh->vertices())
    {
        const Point& zi = template_points[v];
        const Point& xi = template_restpose_points[v];
        Point  trans_xi = affine_transform(global_rigid_trans, xi);

        error_reg_rigid += sqrnorm( zi - trans_xi );
    }
    error_reg_rigid *= (weight_ / nv);

    return error_reg_rigid;
}


//-----------------------------------------------------------------------------


void
Energy_term_rigid_linearized::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;


    // TODO currently not supported
    std::cerr << "Energy_term_rigid_linearized::add_rows_big_lse(...) currently not supported" << std::endl;
    exit(1);


    const unsigned int offset_match = column_offsets.offset_match;
    const unsigned int offset_nrd_w_rigid = column_offsets.offset_nrd_w_rigid;



#ifdef BE_VERBOSE
    std::cerr << "in: 'add_Energy_prior_rigid_3er(...)'" << std::endl;
#endif

    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_rigid = sqrt(weight_ / nv);
    unsigned int i = 0;
    for (auto v : template_mesh->vertices())
    {
        // get sample
        const Point& s = template_restpose_points[v];

        // right hand side
        b(r) = weight_rigid * s[0];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i,             weight_rigid) ); // z_i
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 1, -s[2] * weight_rigid) ); // beta
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 2,  s[1] * weight_rigid) ); // gamma
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 3,        -weight_rigid) ); // t_1
        ++r;

        // right hand side
        b(r) = weight_rigid * s[1];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i + 1,         weight_rigid) ); // z_i
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 0,  s[2] * weight_rigid) ); // alpha
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 2, -s[0] * weight_rigid) ); // gamma
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 4,        -weight_rigid) ); // t_2
        ++r;

        // right hand side
        b(r) = weight_rigid * s[2];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*i + 2,         weight_rigid) ); // z_i
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 0, -s[1] * weight_rigid) ); // alpha
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 1,  s[0] * weight_rigid) ); // beta
        coeffs.push_back( Tripl(r, offset_nrd_w_rigid + 5,        -weight_rigid) ); // t_3
        ++r;

        ++i;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_rigid_linearized::
n_rows_big_sle()
{
    return 3*energy_term_data_.template_mesh->n_vertices();
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
