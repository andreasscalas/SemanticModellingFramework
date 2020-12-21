//== INCLUDES =================================================================


#include "Energy_term_nonlin_aniso_laplace.h"

#include "../settings.h"

#include <graphene/geometry/Matrix4x4.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Energy_term_nonlin_aniso_laplace::
Energy_term_nonlin_aniso_laplace(const Energy_term_data &energy_term_data, double weight) :
    Energy_term(energy_term_data, weight, "Non-Linear Anisotropic Laplace")
{
}


//-----------------------------------------------------------------------------


Energy_term_nonlin_aniso_laplace::
~Energy_term_nonlin_aniso_laplace()
{}


//-----------------------------------------------------------------------------


void
Energy_term_nonlin_aniso_laplace::
pre_minimize_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;
    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
//    auto edge_rotation          = template_mesh->get_edge_property<Mat4f>("e:edge_rotation");
    auto edge_rotation          = template_mesh->get_edge_property<Point>("e:edge_rotation");

    for (auto e : template_mesh->edges())
    {
        edge_rotation[e] = template_aniso_laplace[e];
    }
}


//-----------------------------------------------------------------------------


void
Energy_term_nonlin_aniso_laplace::
pre_sle_assemble_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;
    // std::cerr << "in: 'fix_optimal_rhs_Re_xold_quick(...)'" << std::endl;

    auto template_points        = template_mesh->get_vertex_property<Point>("v:point");
    auto edge_rotation          = template_mesh->get_edge_property<Point>("e:edge_rotation");
    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
    auto area_prop              = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan                  = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop       = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    if ( !edge_rotation          ||
         !template_aniso_laplace ||
         !area_prop              ||
         !cotan                  ||
         !inner_edges_prop )
    {
        std::cerr << "[ERROR] in Energy_term_nonlin_aniso_laplace::pre_sle_assemble_action()" << std::endl;
        return;
    }

    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];
        const double area = area_prop[ inner_edges_prop[0][i] ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // compute laplace(current edge)
        Point lapl_xe(0, 0, 0);
        lapl_xe += wq * template_points[vq];
        lapl_xe += ws * template_points[vs];
        lapl_xe += wr * template_points[vr];
        lapl_xe += wp * template_points[vp];

        Point& old_laplace = template_aniso_laplace[ inner_edges_prop[0][i] ];

        // update local edge rotation R_e x_old
        double factor = norm(old_laplace) / norm(lapl_xe);


        //        std::cerr << "norm(lapl_xe) " << norm(lapl_xe) << std::endl;
        if ( /* isnan(factor) || */ norm(lapl_xe) < 1e-5 ) // numerically instable
        {
            edge_rotation[ inner_edges_prop[0][i] ] = old_laplace; // keep old laplace
        }
        else // good result
        {
            edge_rotation[ inner_edges_prop[0][i] ] = factor * lapl_xe;
        }



        /* ALT
        if ( isnan(factor) )
        {
            factor = 1.0;
        }

        edge_rotation[ inner_edges_prop[0][i] ] = factor * lapl_xe;
*/
    }
}


//-----------------------------------------------------------------------------



double
Energy_term_nonlin_aniso_laplace::
evaluate()
{
    double surface_area_aniso = energy_term_data_.surface_area_aniso;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    auto template_points        = template_mesh->get_vertex_property<Point>("v:point");
    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
//    auto edge_rotation          = template_mesh->get_edge_property<Mat4f>("e:edge_rotation");
    auto edge_rotation          = template_mesh->get_edge_property<Point>("e:edge_rotation");

    auto templ_local_weight     = template_mesh->get_vertex_property<double>("v:regularization_weight");

    double local_weight = 1.0;

    auto area_prop        = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan            = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    const unsigned int ne = inner_edges_prop[0].size();

// BEGIN --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK
/*
    double sigma_hat;
    if ( (robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE) )
    {
        // compute median
        // see: <http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155>
        std::vector<double> res_vec_HACK;
        for ( unsigned int i = 0; i < ne; ++i )
        {
            auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

            auto vp = template_mesh->from_vertex( h );
            auto vq = template_mesh->to_vertex( h );
            auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
            auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

            // cotan values for edge-incident angles
            const double wik = cotan[ h ];
            const double wij = cotan[ template_mesh->prev_halfedge(h) ];
            const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
            const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];
            const double area = area_prop[ inner_edges_prop[0][i] ];

            // weights for edge-based mean curvature vector
            double ws =  (wil + wim) / area;
            double wp = -(wik + wil) / area;
            double wr =  (wik + wij) / area;
            double wq = -(wij + wim) / area;

            // compute laplace(current edge)
            Point lapl_xe(0, 0, 0);
            lapl_xe += wq * template_points[vq];
            lapl_xe += ws * template_points[vs];
            lapl_xe += wr * template_points[vr];
            lapl_xe += wp * template_points[vp];

            // get laplace(xi)
//        const Point& lapl_oldedge = template_aniso_laplace[ inner_edges_prop[0][i] ];
//        const Mat4f& Re = edge_rotation[ inner_edges_prop[0][i] ];
//        Point rotated_old_lapl_edge = affine_transform(Re, lapl_oldedge);
            Point rotated_old_lapl_edge = edge_rotation[ inner_edges_prop[0][i] ];

            double u = norm( lapl_xe - rotated_old_lapl_edge );

            if ( u != 0.0 )
            {
                res_vec_HACK.push_back( u );
            }
            else
            {
                std::cerr << " u is 0  (EVAL NONLINANISO ENERGY)  !!!" << std::endl;
//                exit(1);
            }
        }

        double median_HACK = 0;
        {
            size_t n = res_vec_HACK.size() / 2;
            std::nth_element(res_vec_HACK.begin(), res_vec_HACK.begin()+n, res_vec_HACK.end());
            median_HACK = res_vec_HACK[n];
        }
        std::cerr << "   !!!!!!!!!!!!!!!!!!!!!!!!!!!!            median_HACK (IN EVAL FUNC.): " << median_HACK << std::endl;
        // compute sigma_hat
        sigma_hat = median_HACK; // / 0.675;
//        sigma_hat = 1.0;
        // std::cerr << "SSSIIIGGGMMMAAA: " << sigma_hat << std::endl;
    }
*/
// END   --- TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK TODO HACK

    double error_prior_nonlin_aniso_lapl = 0.0;
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];
        const double area = area_prop[ inner_edges_prop[0][i] ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // compute laplace(current edge)
        Point lapl_xe(0, 0, 0);
        lapl_xe += wq * template_points[vq];
        lapl_xe += ws * template_points[vs];
        lapl_xe += wr * template_points[vr];
        lapl_xe += wp * template_points[vp];

        // get laplace(xi)
//        const Point& lapl_oldedge = template_aniso_laplace[ inner_edges_prop[0][i] ];
//        const Mat4f& Re = edge_rotation[ inner_edges_prop[0][i] ];
//        Point rotated_old_lapl_edge = affine_transform(Re, lapl_oldedge);
        Point rotated_old_lapl_edge = edge_rotation[ inner_edges_prop[0][i] ];


        if (templ_local_weight)
        {
            local_weight = (templ_local_weight[vp] + templ_local_weight[vq]) * Scalar(0.5);
        }


        double u = norm( lapl_xe - rotated_old_lapl_edge );
//        std::cerr << "u: " << u << std::endl;
        if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
        {
            error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * u * u;
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
        {
            error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * std::pow( u, robustness_parameters_.parameter );
        }
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
        {
            double huber_thr = robustness_parameters_.parameter;
            if ( u > huber_thr )
            {
                error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( huber_thr*u - huber_thr*huber_thr/2.0 );
            }
            else
            {
                error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( u*u/2.0 );
            }
        }
/*
        else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
        {
            double scale_parameter = robustness_parameters_.parameter;
//                error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
//                error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3

            // const double aux = std::pow(u/sigma_hat, 2.0);
            // error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( (aux/2.0) / (1.0 + aux/(scale_parameter*scale_parameter)) );

            const double aux = std::pow(u/scale_parameter, 2.0);
            error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * ( (u*u/2) / (1 + aux) );

//////
            // if ( aux > 0.0 )
            // {
                // std::cerr.precision(15);
                // std::cerr << "aux: " << aux << std::endl;
                // std::cerr << "AA: " << area_prop[ inner_edges_prop[0][i] ] * ( (u*u/2) / (1 + aux) ) << std::endl;
                // double tmp_a = area_prop[ inner_edges_prop[0][i] ];
                // std::cerr << "tmp_a: " << tmp_a << std::endl;
                // double tmp_b = u*u;
                // std::cerr << "tmp_b: " << tmp_b << std::endl;
                // double tmp_c = 2.0 * ( 1.0 + tmp_b );
                // std::cerr << "tmp_c: " << tmp_c << std::endl;
                // double tmp_d = tmp_b / tmp_c;
                // std::cerr << "tmp_d: " << tmp_d << std::endl;
                // std::cerr << "ASDASD 4: " << tmp_b / ( 2.0 * ( 1.0 + (tmp_b/(scale_parameter*scale_parameter)) ) ) << std::endl;
            // }
//////
        }
*/
        else // default to L2
        {
            error_prior_nonlin_aniso_lapl += local_weight * area_prop[ inner_edges_prop[0][i] ] * u * u;
        }
    }
    error_prior_nonlin_aniso_lapl *= (weight_ / surface_area_aniso);

    return error_prior_nonlin_aniso_lapl;
}



//-----------------------------------------------------------------------------

void
Energy_term_nonlin_aniso_laplace::
add_rows_small_lse(
        std::vector<Tripl> &coeffs,
        Eigen::MatrixXd &B,
        unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_nonlin_aniso_laplace::add_rows_small_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_nonlin_aniso_laplace::add_rows_small_lgs(...)'" << std::endl;
#endif

    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
//    auto edge_rotation          = template_mesh->get_edge_property<Mat4f>("e:edge_rotation");
    auto edge_rotation          = template_mesh->get_edge_property<Point>("e:edge_rotation");
    auto area_prop              = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan                  = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop       = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    auto templ_local_weight     = template_mesh->get_vertex_property<double>("v:regularization_weight");

    double local_weight = 1.0;

    const double weight_aniso_laplace = sqrt( weight_ / energy_term_data_.surface_area_aniso );
    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        const double area = area_prop[ inner_edges_prop[0][i] ];

//        const Point& lapl_oldedge = template_aniso_laplace[ inner_edges_prop[0][i] ];
//        const Mat4f& Re = edge_rotation[ inner_edges_prop[0][i] ];
//        Point rotated_old_lapl_edge = affine_transform(Re, lapl_oldedge);
        Point rotated_old_lapl_edge = edge_rotation[ inner_edges_prop[0][i] ];

        if (templ_local_weight)
        {
            local_weight = (templ_local_weight[vp] + templ_local_weight[vq]) * Scalar(0.5);
            local_weight = sqrt(local_weight);
        }

        // right hand side
        for ( unsigned int j = 0; j < 3; ++j )
        {
            B(r, j) = weight_aniso_laplace * local_weight * sqrt(area) * rotated_old_lapl_edge[j];
        }

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // scale to weight smoothing vs. fitting. use sqrt of weight
        // since it will be squared for the energy. energy should be
        // sum_i (area_i * norm(laplace_i)), so weight is sqrt(area_i)
        wq  *= weight_aniso_laplace * local_weight * sqrt(area);
        ws  *= weight_aniso_laplace * local_weight * sqrt(area);
        wr  *= weight_aniso_laplace * local_weight * sqrt(area);
        wp  *= weight_aniso_laplace * local_weight * sqrt(area);

        // add coefficients to matrix
        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vq.idx(), wq) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vs.idx(), ws) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vr.idx(), wr) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + vp.idx(), wp) );

        ++r;
    }

//    std::cerr << "out: 'Energy_term_nonlin_aniso_laplace::add_rows_small_lgs(...)'" << std::endl;
}


//-----------------------------------------------------------------------------


void
Energy_term_nonlin_aniso_laplace::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{
    //std::cout << "[INFO] Energy_term_nonlin_aniso_laplace::add_rows_big_lse(): weight_: " << weight_ << std::endl;

    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_nonlin_aniso_laplace::add_rows_big_lgs(...)'" << std::endl;
#endif


    auto template_aniso_laplace = template_mesh->get_edge_property<Point>("e:aniso_laplace");
//    auto edge_rotation          = template_mesh->get_edge_property<Mat4f>("e:edge_rotation");
    auto edge_rotation          = template_mesh->get_edge_property<Point>("e:edge_rotation");
    auto area_prop              = template_mesh->get_edge_property<double>("fitting:area");
    auto cotan                  = template_mesh->get_halfedge_property<double>("fitting:cotan");
    auto inner_edges_prop       = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");

    auto templ_local_weight     = template_mesh->get_vertex_property<double>("v:regularization_weight");

    double local_weight = 1.0;

    const double weight_aniso_laplace = sqrt( weight_ / energy_term_data_.surface_area_aniso );
    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh->from_vertex( h );
        auto vq = template_mesh->to_vertex( h );
        auto vr = template_mesh->to_vertex( template_mesh->next_halfedge(h) );
        auto vs = template_mesh->to_vertex( template_mesh->next_halfedge( template_mesh->opposite_halfedge(h) ) );

        const double area = area_prop[ inner_edges_prop[0][i] ];

//        const Point& lapl_oldedge = template_aniso_laplace[ inner_edges_prop[0][i] ];
//        const Mat4f& Re = edge_rotation[ inner_edges_prop[0][i] ];
//        Point rotated_old_lapl_edge = affine_transform(Re, lapl_oldedge);
        Point rotated_old_lapl_edge = edge_rotation[ inner_edges_prop[0][i] ];

        if (templ_local_weight)
        {
            local_weight = (templ_local_weight[vp] + templ_local_weight[vq]) * Scalar(0.5);
            local_weight = sqrt(local_weight);
        }

        // right hand side
        b(r) = weight_aniso_laplace * local_weight * sqrt(area) * rotated_old_lapl_edge[0];

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh->prev_halfedge( template_mesh->opposite_halfedge(h) ) ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr =  (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // scale to weight smoothing vs. fitting. use sqrt of weight
        // since it will be squared for the energy. energy should be
        // sum_i (area_i * norm(laplace_i)), so weight is sqrt(area_i)
        wq  *= weight_aniso_laplace * local_weight * sqrt(area);
        ws  *= weight_aniso_laplace * local_weight * sqrt(area);
        wr  *= weight_aniso_laplace * local_weight * sqrt(area);
        wp  *= weight_aniso_laplace * local_weight * sqrt(area);

        // add coefficients to matrix
        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vq.idx(), wq) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vs.idx(), ws) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vr.idx(), wr) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vp.idx(), wp) );
        ++r;

        // right hand side
        b(r) = weight_aniso_laplace * local_weight * sqrt(area) * rotated_old_lapl_edge[1];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vq.idx() + 1, wq) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vs.idx() + 1, ws) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vr.idx() + 1, wr) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vp.idx() + 1, wp) );
        ++r;

        // right hand side
        b(r) = weight_aniso_laplace * local_weight * sqrt(area) * rotated_old_lapl_edge[2];

        // begin_row
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vq.idx() + 2, wq) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vs.idx() + 2, ws) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vr.idx() + 2, wr) );
        coeffs.push_back( Tripl(r, column_offsets.offset_match + 3*vp.idx() + 2, wp) );
        ++r;
    }

//    std::cerr << "out: 'Energy_term_nonlin_aniso_laplace::add_rows_big_lgs(...)'" << std::endl;
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_nonlin_aniso_laplace::
n_rows_small_sle()
{
    //std::cout << "[INFO] Energy_term_nonlin_aniso_laplace::n_rows_small_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    auto inner_edges_prop = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    return (unsigned int) inner_edges_prop[0].size();
}


//-----------------------------------------------------------------------------


unsigned int
Energy_term_nonlin_aniso_laplace::
n_rows_big_sle()
{
    //std::cout << "[INFO] Energy_term_nonlin_aniso_laplace::n_rows_big_sle()" << std::endl;

    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    auto inner_edges_prop = template_mesh->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    return (unsigned int) inner_edges_prop[0].size() * 3;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
