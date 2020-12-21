//== INCLUDES =================================================================


#include "Energy_term_eyes_contour.h"

#include "../settings.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_eyes_contour::
Energy_term_eyes_contour(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Eye Contours")
{}


//-----------------------------------------------------------------------------


Energy_term_eyes_contour::
~Energy_term_eyes_contour()
{}


//-----------------------------------------------------------------------------


double
Energy_term_eyes_contour::
evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    const std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    const std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_lefteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_righteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_righteye");
    const std::vector<Surface_mesh::Vertex>& contour_points_left_eye = contour_lefteye[0];
    const std::vector<Surface_mesh::Vertex>& contour_points_right_eye = contour_righteye[0];


    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_match_contoureyes = 0.0;
    const unsigned int C_left = contour_targets_lefteye.size();
    for (unsigned int i = 0; i < C_left; ++i)
    {
        error_match_contoureyes += sqrnorm(contour_targets_lefteye[i] - template_points[contour_points_left_eye[i]]);
    }
    const unsigned int C_right = contour_targets_righteye.size();
    for (unsigned int i = 0; i < C_right; ++i)
    {
        error_match_contoureyes += sqrnorm(contour_targets_righteye[i] - template_points[contour_points_right_eye[i]]);
    }
    error_match_contoureyes *= (weight_ / (C_left+C_right));

    return error_match_contoureyes;
}


//-----------------------------------------------------------------------------


void Energy_term_eyes_contour::
add_rows_small_lse(std::vector<Tripl> &coeffs,
                   Eigen::MatrixXd &B,
                   unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_eyes_contour::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    const std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    const std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_lefteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_righteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_righteye");
    const std::vector<Surface_mesh::Vertex>& contour_points_left_eye = contour_lefteye[0];
    const std::vector<Surface_mesh::Vertex>& contour_points_right_eye = contour_righteye[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int C_left  = contour_targets_lefteye.size();
    const unsigned int C_right = contour_targets_righteye.size();

    const double match_weight_2 = sqrt(weight_  / (C_left+C_right));

    // left eye
    for (unsigned int i = 0; i < C_left; ++i)
    {
        const int idx = contour_points_left_eye[i].idx();
        const Point t = contour_targets_lefteye[i];

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = match_weight_2 * t[j];
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx, match_weight_2) );
        ++r;
    }

    // right eye
    for (unsigned int i = 0; i < C_right; ++i)
    {
        const int idx = contour_points_right_eye[i].idx();
        const Point t = contour_targets_righteye[i];

        // right hand side
        for (unsigned int j = 0; j < 3; ++j)
        {
            B(r, j) = match_weight_2 * t[j];
        }

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + idx, match_weight_2) );
        ++r;
    }
}



//-----------------------------------------------------------------------------
void Energy_term_eyes_contour::
add_rows_big_lse(std::vector<Tripl> &coeffs,
                 Eigen::VectorXd &b,
                 unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_eyes_contour::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    const std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    const std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];

    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_lefteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > contour_righteye
        = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:contour_righteye");
    const std::vector<Surface_mesh::Vertex>& contour_points_left_eye = contour_lefteye[0];
    const std::vector<Surface_mesh::Vertex>& contour_points_right_eye = contour_righteye[0];

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;

    const unsigned int C_left  = contour_targets_lefteye.size();
    const unsigned int C_right = contour_targets_righteye.size();

    const double match_weight_2 = sqrt(weight_  / (C_left+C_right));

    // left eye
    for (unsigned int i = 0; i < C_left; ++i)
    {
        const int idx = contour_points_left_eye[i].idx();
        const Point t = contour_targets_lefteye[i];

        // right hand side
        b(r) = match_weight_2 * t[0];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx, match_weight_2) );
        ++r;

        // right hand side
        b(r) = match_weight_2 * t[1];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 1, match_weight_2) );
        ++r;

        // right hand side
        b(r) = match_weight_2 * t[2];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 2, match_weight_2) );
        ++r;
    }

    // right eye
    for (unsigned int i = 0; i < C_right; ++i)
    {
        const int idx = contour_points_right_eye[i].idx();
        const Point t = contour_targets_righteye[i];

        // right hand side
        b(r) = match_weight_2 * t[0];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx, match_weight_2) );
        ++r;

        // right hand side
        b(r) = match_weight_2 * t[1];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 1, match_weight_2) );
        ++r;

        // right hand side
        b(r) = match_weight_2 * t[2];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*idx + 2, match_weight_2) );
        ++r;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_eyes_contour::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_eyes_contour::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    const std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    const std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];

    const unsigned int num_rows = contour_targets_lefteye.size() + contour_targets_righteye.size();

    return num_rows;
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_eyes_contour::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_eyes_contour::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_lefteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_lefteye");
    const Surface_mesh::Mesh_property< std::vector<Point> > contour_targets_righteye_prop
        = template_mesh->get_mesh_property< std::vector<Point> >("m:contour_targets_righteye");
    const std::vector<Point>& contour_targets_lefteye  = contour_targets_lefteye_prop[0];
    const std::vector<Point>& contour_targets_righteye = contour_targets_righteye_prop[0];

    const unsigned int num_rows = 3*contour_targets_lefteye.size() + 3*contour_targets_righteye.size();

    return num_rows;
}



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
