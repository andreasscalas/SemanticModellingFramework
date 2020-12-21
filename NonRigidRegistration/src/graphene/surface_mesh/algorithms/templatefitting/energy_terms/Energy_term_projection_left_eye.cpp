//== INCLUDES =================================================================


#include "Energy_term_projection_left_eye.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/Triangle_kD_tree.h>

#include "../settings.h"
#include "../utility/my_helper.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_projection_left_eye::
Energy_term_projection_left_eye(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "Projection Left Eye")
{
}


//-----------------------------------------------------------------------------


Energy_term_projection_left_eye::
~Energy_term_projection_left_eye()
{}


//-----------------------------------------------------------------------------


void
Energy_term_projection_left_eye::
pre_minimize_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;

    const auto template_points  = template_mesh->vertex_property<Point>("v:point");
    const auto template_normals = template_mesh->vertex_property<Normal>("v:normal");

    Surface_mesh::Mesh_property< Surface_mesh* > left_eye_mesh_prop
        = template_mesh->get_mesh_property< Surface_mesh* >("m:left_eye_mesh");
    Surface_mesh* left_eye_mesh = left_eye_mesh_prop[0];

    if (!left_eye_mesh)
    {
        std::cerr << "[ERROR]: Energy_term_projection_left_eye::pre_minimize_action(): no eye mesh available." << std::endl;
        return;
    }

    const auto left_eye_mesh_points  = left_eye_mesh->vertex_property<Point>("v:point");
    const auto left_eye_mesh_normals = left_eye_mesh->vertex_property<Normal>("v:normal");

    Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > projection_lefteye_orig_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:projection_lefteye_orig");
    const std::vector<Surface_mesh::Vertex>& projection_left_eye_orig = projection_lefteye_orig_prop[0];

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > projection_lefteye_prop
        = template_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:projection_lefteye");
    std::vector<Surface_mesh::Vertex>& projection_left_eye = projection_lefteye_prop[0];

    projection_targets_lefteye.clear();
    projection_left_eye.clear();

    for ( unsigned int cnt = 0; cnt < projection_left_eye_orig.size(); ++cnt )
    {
        const Point&  point_on_face_template  = template_points[ projection_left_eye_orig[cnt] ];
        const Normal& normal_on_face_template = template_normals[ projection_left_eye_orig[cnt] ];

        // kd tree of eye mesh
        const Triangle_kD_tree left_eye_mesh_kd(*left_eye_mesh);
        // find closest point to sample i on template
        const auto nn_tri = left_eye_mesh_kd.nearest( point_on_face_template );

        const bool is_outside = check_if_outside( point_on_face_template, *left_eye_mesh );

        if ( is_outside ) // point is outside
        {
            Point p_target = nn_tri.nearest;

            projection_targets_lefteye.push_back( p_target );
            projection_left_eye.push_back( projection_left_eye_orig[cnt] );
        }
        else // point is inside
        {
            // determine dot-product of normals
            Normal n_face_template = normal_on_face_template;
            n_face_template.normalize();

            // get vertices of closest triangle
            auto fvit = left_eye_mesh->vertices(nn_tri.face);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);
            const Point p0 = left_eye_mesh_points[v0];
            const Point p1 = left_eye_mesh_points[v1];
            const Point p2 = left_eye_mesh_points[v2];
            const Normal n0 = left_eye_mesh_normals[v0];
            const Normal n1 = left_eye_mesh_normals[v1];
            const Normal n2 = left_eye_mesh_normals[v2];

            // get barycentric coordinates
            const Point b = graphene::geometry::barycentric_coordinates(nn_tri.nearest, p0, p1, p2);
            Normal n_on_eye = b[0]*n0 + b[1]*n1 + b[2]*n2;
            n_on_eye.normalize();
            double dot = graphene::dot( n_face_template, n_on_eye );
//            if ( dot < -0.342 ) // only add if valid
            if ( dot < 0 ) // only add if valid
            {
                Point p_target = nn_tri.nearest;

                projection_targets_lefteye.push_back( p_target );
                projection_left_eye.push_back( projection_left_eye_orig[cnt] );
            }
        }
    }
}


//-----------------------------------------------------------------------------


double
Energy_term_projection_left_eye::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    const std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > projection_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:projection_lefteye");
    const std::vector<Surface_mesh::Vertex>& projection_left_eye = projection_lefteye_prop[0];


    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    // left eye
    double error_match_left_eye_proj = 0.0;
    for (unsigned int i = 0; i < projection_targets_lefteye.size(); ++i)
    {
        error_match_left_eye_proj += sqrnorm(projection_targets_lefteye[i] - template_points[projection_left_eye[i]]);
    }
    error_match_left_eye_proj *= (weight_ / projection_targets_lefteye.size());

    return error_match_left_eye_proj;
}


//-----------------------------------------------------------------------------


void
Energy_term_projection_left_eye::
add_rows_small_lse( std::vector<Tripl>& coeffs,
                    Eigen::MatrixXd& B,
                    unsigned int& r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_match = column_offsets.offset_match;


    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    const std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > projection_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:projection_lefteye");
    const std::vector<Surface_mesh::Vertex>& projection_left_eye = projection_lefteye_prop[0];

  
    const double match_weight_2 = sqrt(weight_ / projection_targets_lefteye.size());

    for (unsigned int i = 0; i < projection_targets_lefteye.size(); ++i)
    {
        const int idx = projection_left_eye[i].idx();
        const Point t = projection_targets_lefteye[i];

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
Energy_term_projection_left_eye::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r )
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_match = column_offsets.offset_match;


    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    const std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > projection_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:projection_lefteye");
    const std::vector<Surface_mesh::Vertex>& projection_left_eye = projection_lefteye_prop[0];


    const double weight_match_2 = sqrt(weight_ / projection_targets_lefteye.size());

    for (unsigned int i = 0; i < projection_targets_lefteye.size(); ++i)
    {
        const int idx = projection_left_eye[i].idx();
        const Point t = projection_targets_lefteye[i];

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
Energy_term_projection_left_eye::
n_rows_small_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    const std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    return (unsigned int) projection_targets_lefteye.size();
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_projection_left_eye::
n_rows_big_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > projection_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:projection_targets_lefteye");
    const std::vector<Point>& projection_targets_lefteye = projection_targets_lefteye_prop[0];

    const unsigned int num_rows = 3*projection_targets_lefteye.size();

    return num_rows;
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
