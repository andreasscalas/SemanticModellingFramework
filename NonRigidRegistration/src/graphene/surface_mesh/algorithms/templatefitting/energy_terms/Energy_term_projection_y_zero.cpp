//== INCLUDES =================================================================


#include "Energy_term_projection_y_zero.h"

#include "../settings.h"
#include "../utility/my_helper.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_projection_y_zero::
Energy_term_projection_y_zero(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Projection Y Zero")
{
}


//-----------------------------------------------------------------------------


Energy_term_projection_y_zero::
~Energy_term_projection_y_zero()
{}


//-----------------------------------------------------------------------------


void
Energy_term_projection_y_zero::
pre_minimize_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;

    const auto template_points  = template_mesh->vertex_property<Point>("v:point");

    Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_to_projectYzero_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    std::vector<Surface_mesh::Vertex>& template_points_to_projectYzero = template_points_to_projectYzero_prop[0];

    projection_targets_yzero.clear();

    for ( unsigned int i = 0; i < template_points_to_projectYzero.size(); ++i )
    {
        const Point& point_on_template = template_points[ template_points_to_projectYzero[i] ];

        const Point target_point(point_on_template[0], 0.0, point_on_template[2]);

        projection_targets_yzero.push_back(target_point);
    }
}


//-----------------------------------------------------------------------------


double
Energy_term_projection_y_zero::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_to_projectYzero_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    std::vector<Surface_mesh::Vertex>& template_points_to_projectYzero = template_points_to_projectYzero_prop[0];


    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_y_zero_proj = 0.0;
    for ( unsigned int i = 0; i < template_points_to_projectYzero.size(); ++i )
    {
        error_y_zero_proj += sqrnorm(projection_targets_yzero[i] - template_points[ template_points_to_projectYzero[i] ]);
    }
    error_y_zero_proj *= (weight_ / projection_targets_yzero.size());

    return error_y_zero_proj;
}


//-----------------------------------------------------------------------------


void
Energy_term_projection_y_zero::
add_rows_small_lse( std::vector<Tripl>& coeffs,
                    Eigen::MatrixXd& B,
                    unsigned int& r)
{
    //std::cout << "[INFO] Energy_term_projection_y_zero::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_match = column_offsets.offset_match;


    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_to_projectYzero_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    std::vector<Surface_mesh::Vertex>& template_points_to_projectYzero = template_points_to_projectYzero_prop[0];


    const double match_weight_2 = sqrt(weight_ / projection_targets_yzero.size());

    for ( unsigned int i = 0; i < template_points_to_projectYzero.size(); ++i )
    {
        const int idx = template_points_to_projectYzero[i].idx();
        const Point t = projection_targets_yzero[i];

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = match_weight_2 * t[j];
        }

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + idx, match_weight_2) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_projection_y_zero::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r )
{
    //std::cout << "[INFO] Energy_term_projection_y_zero::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_match = column_offsets.offset_match;


    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_to_projectYzero_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    std::vector<Surface_mesh::Vertex>& template_points_to_projectYzero = template_points_to_projectYzero_prop[0];


    const double weight_match_2 = sqrt(weight_ / projection_targets_yzero.size());

    for ( unsigned int i = 0; i < template_points_to_projectYzero.size(); ++i )
    {
        const int idx = template_points_to_projectYzero[i].idx();
        const Point t = projection_targets_yzero[i];

        // right hand side
        b(r) = weight_match_2 * t[0];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*idx, weight_match_2) );
        ++r;

        // right hand side
        b(r) = weight_match_2 * t[1];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*idx + 1, weight_match_2) );
        ++r;

        // right hand side
        b(r) = weight_match_2 * t[2];

        // begin_row
        coeffs.push_back( Tripl(r, offset_match + 3*idx + 2, weight_match_2) );
        ++r;
    }
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_projection_y_zero::
n_rows_small_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    const std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    return (unsigned int) projection_targets_yzero.size();
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_projection_y_zero::
n_rows_big_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_yzero_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_yzero");
    const std::vector<Point>& projection_targets_yzero = projection_targets_yzero_prop[0];

    const unsigned int num_rows = 3*projection_targets_yzero.size();

    return num_rows;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
