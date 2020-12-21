//== INCLUDES =================================================================


#include "Energy_term_pca_cpc_ps2mesh.h"

#include "../settings.h"

#include <graphene/geometry/bary_coord.h>

#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_pca_cpc_ps2mesh::
Energy_term_pca_cpc_ps2mesh(const Energy_term_data& energy_term_data, double weight)
: Energy_term(energy_term_data, weight, "PCA CPC PointSet-to-Mesh")
{
}


//-----------------------------------------------------------------------------


Energy_term_pca_cpc_ps2mesh::
~Energy_term_pca_cpc_ps2mesh()
{}

//-----------------------------------------------------------------------------

void
Energy_term_pca_cpc_ps2mesh::
pre_minimize_action()
{
    //skin_correspondences_inverse_ps2mesh(energy_term_data_.template_mesh, energy_term_data_.correspondences, skinned_correspondences_);
    skin_correspondences(energy_term_data_.template_mesh, energy_term_data_.correspondences, skinned_correspondences_, false);
}

//-----------------------------------------------------------------------------


double
Energy_term_pca_cpc_ps2mesh::
evaluate()
{
    const std::vector<Correspondence>& correspondences = energy_term_data_.correspondences;
    const PCA& pca = energy_term_data_.pca;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    double error_reg_pca_cpc_ps2mesh = 0.0;

    const unsigned int C = correspondences.size();

    for (unsigned int i = 0; i < C; ++i)
    {
        // get vertices of closest triangle
        auto fvit = template_mesh->vertices( correspondences[i].face );
        const auto v0 = *fvit;
        const auto v1 = *(++fvit);
        const auto v2 = *(++fvit);
        const Point p0 = template_points[v0];
        const Point p1 = template_points[v1];
        const Point p2 = template_points[v2];
        // get barycentric coordinates
        const Point b = graphene::geometry::barycentric_coordinates( correspondences[i].on_template, p0, p1, p2 );

        int idxA = v0.idx();
        int idxB = v1.idx();
        int idxC = v2.idx();

        double mA_x = pca.m_pca_(3*idxA    );
        double mA_y = pca.m_pca_(3*idxA + 1);
        double mA_z = pca.m_pca_(3*idxA + 2);
        const Point  mA(mA_x, mA_y, mA_z); // TODO EVTL. SCHNELLER ?!

        double mB_x = pca.m_pca_(3*idxB    );
        double mB_y = pca.m_pca_(3*idxB + 1);
        double mB_z = pca.m_pca_(3*idxB + 2);
        const Point  mB(mB_x, mB_y, mB_z); // TODO EVTL. SCHNELLER ?!

        double mC_x = pca.m_pca_(3*idxC    );
        double mC_y = pca.m_pca_(3*idxC + 1);
        double mC_z = pca.m_pca_(3*idxC + 2);
        const Point  mC(mC_x, mC_y, mC_z); // TODO EVTL. SCHNELLER ?!

        Point  PAd(0.0, 0.0, 0.0);
        Point  PBd(0.0, 0.0, 0.0);
        Point  PCd(0.0, 0.0, 0.0);
        for (unsigned int j = 0; j < 3; ++j)
        {
            for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
            {
                PAd[j] += pca.P_pca_(3*idxA + j, dcount) * pca.d_pca_(dcount);
                PBd[j] += pca.P_pca_(3*idxB + j, dcount) * pca.d_pca_(dcount);
                PCd[j] += pca.P_pca_(3*idxC + j, dcount) * pca.d_pca_(dcount);
            }
        }

        double u = norm( b[0]*PAd + b[1]*PBd + b[2]*PCd + b[0]*mA + b[1]*mB + b[2]*mC - correspondences[i].on_ps );
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_reg_pca_cpc_ps2mesh += u * u;
        }
        else
        {
            std::cerr << "[ERROR] in 'Energy_term_pca_cpc_ps2mesh::evaluate()' robust function not supported! Aborting..." << std::endl;
            exit(1);
        }
    }
    error_reg_pca_cpc_ps2mesh *= (weight_ / C);

    return error_reg_pca_cpc_ps2mesh;
}


//-----------------------------------------------------------------------------


void
Energy_term_pca_cpc_ps2mesh::
add_rows_big_lse( std::vector<Tripl>& coeffs,
                  Eigen::VectorXd& b,
                  unsigned int& r)
{
    //std::cerr << "[INFO] in: Energy_term_pca_cpc_ps2mesh::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    //choose either the skinnend correspondences or the "normal" ones based on whether skinned correspondences exist or not
    const std::vector<Correspondence>& correspondences = skinned_correspondences_.empty() ? energy_term_data_.correspondences : skinned_correspondences_;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const unsigned int offset_nrd_w_pca = column_offsets.offset_nrd_w_pca;
    const PCA& pca = energy_term_data_.pca;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points = template_mesh->get_vertex_property<Point>("v:point");

    const unsigned int C = correspondences.size();

    const double weight_model = sqrt(weight_ / C);
    //std::cout << "[INFO] Energy_term_pca_cpc_ps2mesh::add_rows_big_lse(): weight_model: " << weight_model << std::endl;
    for (unsigned int i = 0; i < C; ++i)
    {
        if (correspondences[i].constr_dir == graphene::surface_mesh::corresp_dir_mesh2ps ||
            correspondences[i].constr_dir == graphene::surface_mesh::corresp_dir_mesh2ps_and_ps2mesh ) // TODO EVTL. WOANDERS HIN
        {
            std::cerr << "[ERROR] in 'Energy_term_pca_cpc_ps2mesh::add_rows_big_lse()' bad exception! Aborting..." << std::endl;
            exit(1);
        }

        // get vertices of closest triangle
        auto fvit = template_mesh->vertices( correspondences[i].face );
        const auto v0 = *fvit;
        const auto v1 = *(++fvit);
        const auto v2 = *(++fvit);
        const Point p0 = template_points[v0];
        const Point p1 = template_points[v1];
        const Point p2 = template_points[v2];
        // get barycentric coordinates
        const Point bary = correspondences[i].on_template_bc;//graphene::geometry::barycentric_coordinates( correspondences[i].on_template, p0, p1, p2 );

        int idxA = v0.idx();
        int idxB = v1.idx();
        int idxC = v2.idx();

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[0] - bary[0]*pca.m_pca_(3*idxA + 0) - bary[1]*pca.m_pca_(3*idxB + 0) - bary[2]*pca.m_pca_(3*idxC + 0) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, ( bary[0]*pca.P_pca_(3*idxA, dcount) + bary[1]*pca.P_pca_(3*idxB, dcount) + bary[2]*pca.P_pca_(3*idxC, dcount) ) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[1] - bary[0]*pca.m_pca_(3*idxA + 1) - bary[1]*pca.m_pca_(3*idxB + 1) - bary[2]*pca.m_pca_(3*idxC + 1) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, ( bary[0]*pca.P_pca_(3*idxA + 1, dcount) + bary[1]*pca.P_pca_(3*idxB + 1, dcount) + bary[2]*pca.P_pca_(3*idxC + 1, dcount) ) * weight_model) ); // d's
        }
        ++r;

        // right hand side
        b(r) = weight_model * ( correspondences[i].on_ps[2] - bary[0]*pca.m_pca_(3*idxA + 2) - bary[1]*pca.m_pca_(3*idxB + 2) - bary[2]*pca.m_pca_(3*idxC + 2) );

        // begin_row
        for (unsigned int dcount = 0; dcount < pca.dim_pca_model_; ++dcount)
        {
            coeffs.push_back( Tripl(r, offset_nrd_w_pca + dcount, ( b[0]*pca.P_pca_(3*idxA + 2, dcount) + b[1]*pca.P_pca_(3*idxB + 2, dcount) + b[2]*pca.P_pca_(3*idxC + 2, dcount) ) * weight_model) ); // d's
        }
        ++r;
    }
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_pca_cpc_ps2mesh::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_pca_cpc_ps2mesh::n_rows_big_sle()" << std::endl;

    return (unsigned int)3*energy_term_data_.correspondences.size();
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
