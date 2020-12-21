//== INCLUDES =================================================================


#include "Energy_term_arap.h"

#include "../settings.h"

#include <graphene/geometry/Matrix4x4.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::Mat4;


//== IMPLEMENTATION ===========================================================


Energy_term_arap::
Energy_term_arap(const Energy_term_data& energy_term_data, double weight)  :
    Energy_term(energy_term_data, weight, "ARAP")
{
}


//-----------------------------------------------------------------------------


Energy_term_arap::
~Energy_term_arap()
{}


//-----------------------------------------------------------------------------

void
Energy_term_arap::
pre_minimize_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;
    auto localrotPOS  = template_mesh->vertex_property<Mat4f>("v:localrotationPOS");
    for (auto v : template_mesh->vertices())
    {
        localrotPOS[v] = Mat4f::identity();
    }
}

//-----------------------------------------------------------------------------

void
Energy_term_arap::
pre_sle_assemble_action()
{
    Surface_mesh *template_mesh = energy_term_data_.template_mesh;
    // std::cerr << "in: 'fix_optimal_rotations_SVD(...)'" << std::endl;

    auto template_points      = template_mesh->vertex_property<Point>("v:point");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");
    auto localrotPOS          = template_mesh->get_vertex_property<Mat4f>("v:localrotationPOS");

    if (!template_restpose_points || !cotan || !localrotPOS)
    {
        std::cerr << "[ERROR] in Energy_term_arap::pre_sle_assemble_action()" << std::endl;
        return;
    }

    for (auto v : template_mesh->vertices())
    {
        // compute data
        std::vector<Point>  eij_old;
        std::vector<Point>  eij_new;
        std::vector<double> wij;

        for (auto hc : template_mesh->halfedges(v))
        {
            Surface_mesh::Edge    cur_e = template_mesh->edge(hc); // TODO KONTROLLIEREN
            Surface_mesh::Vertex  vj    = template_mesh->to_vertex(hc);
            const double cur_wij        = cotan[cur_e] / 2.0;
            const Point  new_e          = template_points[v]  - template_points[vj];
            const Point  old_e          = template_restpose_points[v] - template_restpose_points[vj];

            eij_old.push_back(old_e);
            eij_new.push_back(new_e);
            wij.push_back(cur_wij);
        }

        unsigned int num_cols = eij_old.size();
        if (eij_old.size() != eij_new.size() && eij_old.size() != wij.size())
        {
            std::cerr << "[ERROR] in Energy_term_arap::pre_sle_assemble_action()" << std::endl;
            return;
        }

        // build Pi
        Eigen::MatrixXd  Pi_old(3, num_cols);
        for (unsigned int c = 0; c < eij_old.size(); ++c)
        {
            Pi_old(0, c) = eij_old[c][0];
            Pi_old(1, c) = eij_old[c][1];
            Pi_old(2, c) = eij_old[c][2];
        }

        // build P'i
        Eigen::MatrixXd  Pi_new(3, num_cols);
        for (unsigned int c = 0; c < eij_new.size(); ++c)
        {
            Pi_new(0, c) = eij_new[c][0];
            Pi_new(1, c) = eij_new[c][1];
            Pi_new(2, c) = eij_new[c][2];
        }

        // build Di
        Eigen::MatrixXd  Di(num_cols, num_cols);
        Di.setIdentity();
        for (unsigned int c = 0; c < wij.size(); ++c)
        {
            Di(c, c) = wij[c];
        }

        // build Si
        Eigen::MatrixXd  Si = Pi_old * Di * Pi_new.transpose();
        //        Eigen::MatrixXd  Si_untouched = Si; // TODO

        // perform SVD on Si
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Si, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //        //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;

        // build Ri
        Eigen::MatrixXd  Ri = svd.matrixV() * svd.matrixU().transpose();
        double det_Ri = Ri(0,0)*Ri(1,1)*Ri(2,2) + Ri(0,1)*Ri(1,2)*Ri(2,0) + Ri(0,2)*Ri(1,0)*Ri(2,1) - Ri(2,0)*Ri(1,1)*Ri(0,2) - Ri(2,1)*Ri(1,2)*Ri(0,0) - Ri(2,2)*Ri(1,0)*Ri(0,1);
        if(det_Ri < 0.0) // TODO VLLT. MIT EIGENS DETERMINANTENBERECHNUNG
        {
            Eigen::MatrixXd  Umod = svd.matrixU();
            Umod(0,2) *= -1.0;
            Umod(1,2) *= -1.0;
            Umod(2,2) *= -1.0;
            Ri = svd.matrixV() * Umod.transpose();
            det_Ri = Ri(0,0)*Ri(1,1)*Ri(2,2) + Ri(0,1)*Ri(1,2)*Ri(2,0) + Ri(0,2)*Ri(1,0)*Ri(2,1) - Ri(2,0)*Ri(1,1)*Ri(0,2) - Ri(2,1)*Ri(1,2)*Ri(0,0) - Ri(2,2)*Ri(1,0)*Ri(0,1);
        }
        // std::cerr << "det_Ri: " << det_Ri << std::endl;

        // store Ri
        Mat4f the_opt_Ri = Mat4f::identity();
        the_opt_Ri(0,0) = Ri(0,0);
        the_opt_Ri(0,1) = Ri(0,1);
        the_opt_Ri(0,2) = Ri(0,2);
        the_opt_Ri(1,0) = Ri(1,0);
        the_opt_Ri(1,1) = Ri(1,1);
        the_opt_Ri(1,2) = Ri(1,2);
        the_opt_Ri(2,0) = Ri(2,0);
        the_opt_Ri(2,1) = Ri(2,1);
        the_opt_Ri(2,2) = Ri(2,2);


        //TODO reenable this switch?
        if (true)
        {
            /*
// compute barycenters
Vec3f scog(0.0,0.0,0.0), dcog(0.0,0.0,0.0);
unsigned int valence = 0;
hc = template_mesh->halfedges(v);
hc_end = hc;
do
{
Surface_mesh::Vertex  vj    = template_mesh->to_vertex(*hc);

scog += template_restpose_points[vj];
dcog += template_points[vj];

valence++;
}
while (++hc != hc_end);
scog /= (float) valence;
dcog /= (float) valence;
*/





            // VERSION HORN VARIANTE ODER SO / BZW. PAPAZOV 2011
            Vec3f scog = template_restpose_points[v]; // TODO BESSER MACHEN
            Vec3f dcog = template_points[v];

            Vec3f  sp, dp;
            float  nom(0), denom(0);

            for (auto hc : template_mesh->halfedges(v))
            {
                Surface_mesh::Vertex  vj    = template_mesh->to_vertex(hc);
                sp = template_restpose_points[vj]; sp -= scog;
                dp = template_points[vj]; dp -= dcog;

                nom   += dot(dp,dp);
                denom += dot(sp,sp);
            }

            const float scaling  = sqrt( nom  / denom );

            //            std::cerr << "In fix_optimal_rotations_SVD: SCALING  : " << scaling  << std::endl;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    the_opt_Ri(i, j) *= scaling;
                }
            }


            /*
// VERSION VORHER: Von Graphenes Code abgeguckt
Vec3f scog = template_restpose_points[v]; // TODO BESSER MACHEN
Vec3f dcog = template_points[v];

Vec3f  sp, dp;
float  nom(0), denom(0);

hc = template_mesh->halfedges(v);
hc_end = hc;
do
{
Surface_mesh::Edge    cur_e = template_mesh->edge(*hc);
Surface_mesh::Vertex  vj    = template_mesh->to_vertex(*hc);
sp = template_restpose_points[vj]; sp -= scog;
dp = template_points[vj]; dp -= dcog;

sp = linear_transform(the_opt_Ri, sp);

nom   += dot(sp,dp);
denom += dot(sp,sp);
}
while (++hc != hc_end);

const float scaling  = nom  / denom;

//            std::cerr << "In fix_optimal_rotations_SVD: SCALING  : " << scaling  << std::endl;
for (int i = 0; i < 3; ++i)
{
for (int j = 0; j < 3; ++j)
{
the_opt_Ri(i, j) *= scaling;
}
}
*/

        }

        // update local rotation R_i
        localrotPOS[v] = the_opt_Ri;
    }
}

//-----------------------------------------------------------------------------

double Energy_term_arap::evaluate()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

    const unsigned int nv = template_mesh->n_vertices();

    auto template_points      = template_mesh->get_vertex_property<Point>("v:point");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");
    auto localrotPOS          = template_mesh->get_vertex_property<Mat4f>("v:localrotationPOS");

    double error_prior_arap = 0.0;
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            const Point& xi = template_restpose_points[v];
            const Point& xj = template_restpose_points[template_mesh->to_vertex(one_ring[or_it])];
            const Point& zi = template_points[v];
            const Point& zj = template_points[template_mesh->to_vertex(one_ring[or_it])];
            const Mat4f& Ri = localrotPOS[v];
            Point rotated_xji = linear_transform(Ri, xj - xi);

            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            double weight_local_vertex = cotan[curr_edg] / 2.0;

            double u = norm( (zj - zi) - rotated_xji );
            if ( robustness_parameters_.solver_type == SOLVER_TYPE_L2 )
            {
                error_prior_arap += weight_local_vertex * u * u;
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_L1 )
            {
                error_prior_arap += weight_local_vertex * std::pow( u, robustness_parameters_.parameter );
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_HUBER )
            {
                double huber_thr = robustness_parameters_.parameter;
                if ( u > huber_thr )
                {
                    error_prior_arap += weight_local_vertex * ( huber_thr*u - huber_thr*huber_thr/2.0 );
                }
                else
                {
                    error_prior_arap += weight_local_vertex * ( u*u/2.0 );
                }
            }
            else if ( robustness_parameters_.solver_type == SOLVER_TYPE_GEMANMCCLURE )
            {
                double scale_parameter = robustness_parameters_.parameter;
//                    error_prior_arap += weight_local_vertex * ( u*u / (scale_parameter*scale_parameter + u*u) ); // VERSION 2
                error_prior_arap += weight_local_vertex * ( ( u*u / ( 2.0 * ( 1.0 + (u*u/(scale_parameter*scale_parameter)) ) ) ) ); // VERSION 3
            }
            else // default to L2
            {
                error_prior_arap += weight_local_vertex * u * u;
            }
        }
    }
    error_prior_arap *= (weight_ / nv); // TODO !!!!!!!!! PASST DAS ?!

    return error_prior_arap;
}

//-----------------------------------------------------------------------------

void Energy_term_arap::
add_rows_small_lse(
        std::vector<Tripl> &coeffs,
        Eigen::MatrixXd &B,
        unsigned int &r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_arap::add_rows_small_lgs_arap_alter(...)'" << std::endl;
#endif

    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto localrotPOS          = template_mesh->get_vertex_property<Mat4f>("v:localrotationPOS");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_arap = sqrt(weight_ / nv);
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            Surface_mesh::Vertex     vj = template_mesh->to_vertex(one_ring[or_it]);
            double weight_local_vertex  = cotan[curr_edg] / 2.0;
            weight_local_vertex = sqrt(weight_local_vertex);

            const Vec3   ad         = (template_restpose_points[vj] - template_restpose_points[v]);
            const Mat4f& current_Ri = localrotPOS[v];
            const Vec3   ad_rotated = linear_transform(current_Ri, ad);

            // right hand side
            for (unsigned int j = 0; j < 3; ++j)
            {
                B(r, j) = weight_arap * weight_local_vertex * (ad_rotated[j]);
            }

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + template_mesh->to_vertex(one_ring[or_it]).idx(), weight_arap * weight_local_vertex) ); // z_j
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + v.idx(),                                       - weight_arap * weight_local_vertex) ); // z_i
            ++r;
        }
    }
}



//-----------------------------------------------------------------------------

void
Energy_term_arap::
add_rows_big_lse(
        std::vector<Tripl> &coeffs,
        Eigen::VectorXd &b,
        unsigned int &r)
{
    const Column_offsets& column_offsets = energy_term_data_.column_offsets;
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;

#ifdef BE_VERBOSE
    std::cerr << "in: 'Energy_term_arap::add_rows_big_lgs_arap_alter(...)'" << std::endl;
#endif

    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");
    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
    auto localrotPOS             = template_mesh->get_vertex_property<Mat4f>("v:localrotationPOS");

    const unsigned int nv = template_mesh->n_vertices();

    const double weight_arap = sqrt(weight_ / nv);
    for (auto v : template_mesh->vertices())
    {
        // get neighbour handles of sample s (store one-ring)
        std::vector<Surface_mesh::Halfedge> one_ring;
        for (auto hv : template_mesh->halfedges(v))
        {
            one_ring.push_back(hv);
        }

        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
        {
            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
            Surface_mesh::Vertex     vj = template_mesh->to_vertex(one_ring[or_it]);
            double weight_local_vertex  = cotan[curr_edg] / 2.0;
            weight_local_vertex = sqrt(weight_local_vertex);

            const Vec3   ad         = (template_restpose_points[vj] - template_restpose_points[v]);
            const Mat4f& current_Ri = localrotPOS[v];
            const Vec3   ad_rotated = linear_transform(current_Ri, ad);

            // right hand side
            b(r) = weight_arap * weight_local_vertex * ad_rotated[0];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx(), weight_arap * weight_local_vertex) ); // z_j
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*v.idx(),                                       - weight_arap * weight_local_vertex) ); // z_i
            ++r;

            // right hand side
            b(r) = weight_arap * weight_local_vertex * ad_rotated[1];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 1,  weight_arap * weight_local_vertex) ); // z_j
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*v.idx() + 1                                        , -weight_arap * weight_local_vertex) ); // z_i
            ++r;

            // right hand side
            b(r) = weight_arap * weight_local_vertex * ad_rotated[2];

            // begin_row
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 2, weight_arap * weight_local_vertex) ); // z_j
            coeffs.push_back( Tripl(r, column_offsets.offset_match      + 3*v.idx() + 2,                                        -weight_arap * weight_local_vertex) ); // z_i
            ++r;
        }
    }
}



//-----------------------------------------------------------------------------


unsigned int
Energy_term_arap::
n_rows_small_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    unsigned int n = 0;
    for (auto v : template_mesh->vertices())
    {
        for (auto hv : template_mesh->halfedges(v))
        {
            ++n;
        }
    }
    return n;
}

//-----------------------------------------------------------------------------


unsigned int
Energy_term_arap::
n_rows_big_sle()
{
    const Surface_mesh* template_mesh = energy_term_data_.template_mesh;
    unsigned int n = 0;
    for (auto v : template_mesh->vertices())
    {
        for (auto hv : template_mesh->halfedges(v))
        {
            n += 3;
        }
    }
    return n;
}

//-----------------------------------------------------------------------------


//void
//Energy_term_arap::
//add_rows_big_lgs_arap_linearized( const double reg_weight, std::vector<Tripl>& coeffs, Eigen::VectorXd& b, unsigned int& r const unsigned int offset_match, const unsigned int offset_nrd_w_arap, Surface_mesh* template_mesh )
//{
//#ifdef BE_VERBOSE
//    std::cerr << "in: 'Energy_term_arap::add_rows_big_lgs_arap_linearized(...)'" << std::endl;
//#endif

//    auto cotan                = template_mesh->get_edge_property<double>("e:cotan");
//    auto template_restpose_points = template_mesh->get_vertex_property<Point>("v:rest_pose");
//    auto localrotPOS             = template_mesh->get_vertex_property<Mat4f>("v:localrotationPOS");

//    const double weight_arap = sqrt(reg_weight);
//    unsigned int i = 0;
//    for (auto v : template_mesh->vertices())
//    {
//        // get neighbour handles of sample s (store one-ring)
//        std::vector<Surface_mesh::Halfedge> one_ring;
//        for (auto hv : template_mesh->halfedges(v))
//        {
//            one_ring.push_back(hv);
//        }

//        for (unsigned int or_it = 0; or_it < one_ring.size(); ++or_it)
//        {
//            Surface_mesh::Edge curr_edg = template_mesh->edge(one_ring[or_it]);
//            Surface_mesh::Vertex     vj = template_mesh->to_vertex(one_ring[or_it]);
//            double weight_local_vertex  = cotan[curr_edg] / 2.0;
//            weight_local_vertex = sqrt(weight_local_vertex);

//            const Vec3           ad = (template_restpose_points[vj] - template_restpose_points[v]);
//            const Mat4f& current_Ri = localrotPOS[v];
//            const Vec3   ad_rotated = linear_transform(current_Ri, ad);

//            // right hand side
//            b(r) = weight_arap * weight_local_vertex * ad_rotated[0];

//            // begin_row
//            coeffs.push_back( Tripl(r, offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx(), weight_arap * weight_local_vertex) ); // z_j
//            coeffs.push_back( Tripl(r, offset_match      + 3*i,                                              - weight_arap * weight_local_vertex) ); // z_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 1,                                 weight_arap * weight_local_vertex * (-ad_rotated[2])) ); // beta_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 2,                                 weight_arap * weight_local_vertex * ( ad_rotated[1])) ); // gamma_i
//            ++r;

//            // right hand side
//            b(r) = weight_arap * weight_local_vertex * ad_rotated[1];

//            // begin_row
//            coeffs.push_back( Tripl(r, offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 1,  weight_arap * weight_local_vertex) ); // z_j
//            coeffs.push_back( Tripl(r, offset_match      + 3*i + 1                                               , -weight_arap * weight_local_vertex) ); // z_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 0                                    ,  weight_arap * weight_local_vertex * ( ad_rotated[2])) ); // alpha_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 2                                    ,  weight_arap * weight_local_vertex * (-ad_rotated[0])) ); // gamma_i
//            ++r;

//            // right hand side
//            b(r) = weight_arap * weight_local_vertex * ad_rotated[2];

//            // begin_row
//            coeffs.push_back( Tripl(r, offset_match      + 3*template_mesh->to_vertex(one_ring[or_it]).idx() + 2, weight_arap * weight_local_vertex) ); // z_j
//            coeffs.push_back( Tripl(r, offset_match      + 3*i + 2,                                               -weight_arap * weight_local_vertex) ); // z_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i    ,                                   weight_arap * weight_local_vertex * ( -ad_rotated[1] )) ); // alpha_i
//            coeffs.push_back( Tripl(r, offset_nrd_w_arap + 3*i + 1,                                   weight_arap * weight_local_vertex * (  ad_rotated[0] )) ); // beta_i
//            ++r;
//        }

//        ++i;
//    }
//}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
