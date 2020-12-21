//== INCLUDES ===================================================================


#include "Example_based_facial_rigging.h"

#include <graphene/geometry/Matrix3x3.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/diffgeo.h>

#include <cfloat>

//#include <iostream>

//#include "/usr/include/stdafx.h"
//#include "/usr/include/optimization.h"


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//using namespace alglib;


//== IMPLEMENTATION ============================================================   

void 
Example_based_facial_rigging::
create_deformed_targets(const Deformation_transfer_data& data)
{   
    // load neutral individual blend shape 'B_0' (complete mesh)
    mesh_bs_B0_.read(data.target_undeformed.c_str() );    
    num_vertices_ = mesh_bs_B0_.n_vertices();

    // load generic neutral blend shape 'A_0' (complete mesh)
    generic_bs_mesh_A0_.read(data.source_undeformed.c_str() );
    check_number_vertices( generic_bs_mesh_A0_);

    // load generic blend shape model 'A_1,...,A_n' (with 'm < n') (complete meshes)
    std::set<std::string>::iterator it_file = data.source_deformed.begin();
    while (it_file != data.source_deformed.end())
    {
        Surface_mesh mesh_Ai;
        mesh_Ai.read(it_file->c_str());
        check_number_vertices( mesh_Ai );
        generic_bs_meshes_Ai_.push_back( mesh_Ai );
        ++it_file;
    }
    
    // load 'm' example poses 'S_j' (complete meshes)
    it_file = data.examples.begin();
    while (it_file != data.examples.end())
    {      
        Surface_mesh example_mesh;
        example_mesh.read(it_file->c_str());
        check_number_vertices(example_mesh);
        example_meshes_Sj_.push_back(example_mesh);
        ++it_file;
    }

    // load weights
    const unsigned int m = example_meshes_Sj_.size();
    const unsigned int n = generic_bs_meshes_Ai_.size();
    coeffs_aij_ = Eigen::MatrixXd (n, m);
    for ( unsigned i = 0; i < n; ++i )
    {
        for ( unsigned j = 0; j < m; ++j )
        {            
            coeffs_aij_(i, j) = data.examples_weights[i][j];
        }
    }        
    generate_blendshapes(data);
}

//-----------------------------------------------------------------------------

Example_based_facial_rigging::
~Example_based_facial_rigging()
{

}

//-----------------------------------------------------------------------------

void
Example_based_facial_rigging::
generate_blendshapes(const Deformation_transfer_data& data)
{
    std::cout << "in 'Example_based_facial_rigging::generate_blendshapes(...)'" << std::endl;

    double beta  = 0.30;
    double gamma = 1000.0;

    const double beta_decr_factor  = 0.85133992252; // from 0.5 to 0.1 in 10 iterations
    const double gamma_decr_factor = 0.79432823472; // from 1000 to 100 in 10 iterations

    const unsigned int m = example_meshes_Sj_.size();
    const unsigned int n = generic_bs_meshes_Ai_.size();
    std::cout << "m: " << m << std::endl;
    std::cout << "n: " << n << std::endl;

    Surface_mesh::Vertex_property<Point> points_mesh_bs_B0_ = mesh_bs_B0_.vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> points_mesh_A0_ = generic_bs_mesh_A0_.vertex_property<Point>("v:point");


    //const unsigned int num_iters = 10;
    //for ( unsigned int current_iter = 0; current_iter < num_iters; ++current_iter )
    //{
    //    std::cerr << "current_iter: " << current_iter << std::endl;

        // Step A: Fix 'a_ij' and solve for 'B_i'
        std::cout << "Step A: Fix 'a_ij' and solve for 'B_i'" << std::endl;

        // minimize total energy for all triangles independently
        std::vector< Eigen::MatrixXd > solutions_X;
        for ( auto f_it : mesh_bs_B0_.faces() )
        {
            // local frame M_B_0
            Eigen::MatrixXd M_B_0(3, 2);
            std::vector<Eigen::Vector3d> vertices;
            for ( auto vf_it : mesh_bs_B0_.vertices(f_it) )
            {
                Point v = points_mesh_bs_B0_[vf_it];
                vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
            }
            M_B_0.block<3, 1>( 0, 0 ) = vertices[1] - vertices[0];
            M_B_0.block<3, 1>( 0, 1 ) = vertices[2] - vertices[0];
            //? normal is missing ??

            // setup matrix
            SpMat A_full(3*m + 3*n, 3*n);
            Eigen::MatrixXd B_full(3*m + 3*n, 2);
            std::vector< Tripl >  coeffs_A;

            // fitting energy - local frame should match examples
            for ( unsigned int j = 0; j < m; ++j ) //all example meshes
            {
                // matrix A - blendshape weights for example  //?? check matrix construction
                for ( unsigned int i = 0; i < n; ++i ) //all blendshapes
                {
                    coeffs_A.push_back( Tripl( 3*j + 0, 3*i + 0, coeffs_aij_(i, j) ) );
                    coeffs_A.push_back( Tripl( 3*j + 1, 3*i + 1, coeffs_aij_(i, j) ) );
                    coeffs_A.push_back( Tripl( 3*j + 2, 3*i + 2, coeffs_aij_(i, j) ) );
                }

                // local frame M_S_j - local frame from examples
                Eigen::MatrixXd M_S_j(3, 2);
                auto points_mesh_example_Sj = example_meshes_Sj_[j].vertex_property<Point>("v:point");
                std::vector<Eigen::Vector3d> vertices;
                for ( auto vf_it : example_meshes_Sj_[j].vertices(f_it) )
                {
                    Point v = points_mesh_example_Sj[vf_it];
                    vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
                }
                M_S_j.block<3, 1>( 0, 0 ) = vertices[1] - vertices[0];
                M_S_j.block<3, 1>( 0, 1 ) = vertices[2] - vertices[0];

                // rhs matrix B
                B_full.block< 3, 2 >( 3*j, 0 ) = M_S_j - M_B_0;  //?? M_B_0-M_S_j
            }

            // compute deformation gradient G_A0_Ai
            // local frame M_A_0 and inverse
            Eigen::Matrix3d M_A_0;
            {
               std::vector<Eigen::Vector3d> vertices;
               for (auto vf_it : generic_bs_mesh_A0_.vertices(f_it))
               {
                  Point v = points_mesh_A0_[vf_it];
                  vertices.push_back(Eigen::Vector3d(v[0], v[1], v[2]));
               }
               M_A_0.block<3, 1>(0, 0) = vertices[1] - vertices[0];
               M_A_0.block<3, 1>(0, 1) = vertices[2] - vertices[0];
               Eigen::Vector3d n = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
               M_A_0.block<3, 1>(0, 2) = n;
            }
            Eigen::Matrix3d M_A_0_inv = M_A_0.inverse();


            // regularization energy - deformation gradients should be similar
            for ( unsigned int i = 0; i < n; ++i )
            {               
                // local frame M_A_i
                Eigen::Matrix3d M_A_i;
                auto points_mesh_generic_bs_i = generic_bs_meshes_Ai_[i].vertex_property<Point>("v:point");
                {
                    std::vector<Eigen::Vector3d> vertices;
                    for ( auto vf_it : generic_bs_meshes_Ai_[i].vertices(f_it) )
                    {
                        Point v = points_mesh_generic_bs_i[vf_it] - points_mesh_A0_[vf_it];
                        vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
                    }
                    M_A_i.block<3, 1>( 0, 0 ) = vertices[1] - vertices[0];
                    M_A_i.block<3, 1>( 0, 1 ) = vertices[2] - vertices[0];
                    Eigen::Vector3d n = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
                    M_A_i.block<3, 1>( 0, 2 ) = n;
                }
                // deformation gradient G_A0_Ai
                Eigen::Matrix3d G_A0_AI = (M_A_0 + M_A_i) *M_A_0_inv;

                // compute weight w_i
                const double norm_M_A_i = M_A_i.norm();
                const double kappa = 0.1;
                const double theta = 2.0;
                const double num   = 1.0 + norm_M_A_i;
                const double denom = kappa + norm_M_A_i;
                const double w_i = std::pow( num/denom, theta );
                const double w_i_sqrt = sqrt( w_i );
                const double beta_sqrt = sqrt( beta );

                // matrix A
                //coeffs_A.push_back( Tripl( 3*m + 3*i + 0, 3*i + 0, w_i_sqrt * beta_sqrt ) );
                //coeffs_A.push_back( Tripl( 3*m + 3*i + 1, 3*i + 1, w_i_sqrt * beta_sqrt ) );
                //coeffs_A.push_back( Tripl( 3*m + 3*i + 2, 3*i + 2, w_i_sqrt * beta_sqrt ) );

                coeffs_A.push_back(Tripl(3 * m + 3 * i + 0, 3 * i + 0, w_i * beta));
                coeffs_A.push_back(Tripl(3 * m + 3 * i + 1, 3 * i + 1, w_i * beta));
                coeffs_A.push_back(Tripl(3 * m + 3 * i + 2, 3 * i + 2, w_i * beta));


                // rhs matrix B
                //Eigen::MatrixXd rhs_reg = w_i_sqrt * beta_sqrt * ( G_A0_AI*M_B_0 - M_B_0 ); //correct weighting
                Eigen::MatrixXd rhs_reg = w_i * beta * (G_A0_AI*M_B_0 - M_B_0); //correct weighting
                //Eigen::MatrixXd rhs_reg = 1 * ( G_A0_AI*M_B_0 - M_B_0 ); //test weighting
                if ( (rhs_reg.rows() != 3) ||
                     (rhs_reg.cols() != 2) )
                {
                    std::cerr << "ERROR rhs_reg" << std::endl;
                }
                B_full.block< 3, 2 >( 3*m + 3*i, 0 ) = rhs_reg;
            }


            // solve the linear system of equations
            A_full.setFromTriplets( coeffs_A.begin(), coeffs_A.end() );
            SpMat AtA = A_full.transpose() * A_full;
            Eigen::MatrixXd AtB = A_full.transpose() * B_full;
            Eigen::SparseSPDSolver< SpMat > solver;
            solver.compute(AtA);
            if ( solver.info() != Eigen::Success )
            {
                // decomposition failed
                std::cerr << "decomposition failed" << std::endl;
                return;
            }
            Eigen::MatrixXd X = solver.solve(AtB);
            if ( solver.info() != Eigen::Success )
            {
                // solving failed
                std::cerr << "solving failed" << std::endl;
                return;
            }
            if ( (X.rows() != 3*n) ||
                 (X.cols() != 2) )
            {
                std::cerr << "ERROR gbgklAEG" << std::endl;
                return;
            }

            solutions_X.push_back( X );
        
        } //end optimizations local frames


        // solve for vertex positions for each blend shape 'B_1,...,B_n'
        std::vector< Surface_mesh > results_B_i;
        for ( unsigned int i = 0; i < n; ++i )
        {
            Surface_mesh result_B_i = mesh_bs_B0_;

            auto points_mesh_result_B_i = result_B_i.vertex_property<Point>("v:point");

            // compute edge and vertex weights
            auto tar_undef_weight_cotan = result_B_i.edge_property<double>("e:tar_undef_cotan");
            for ( auto e : result_B_i.edges() )
            {
                tar_undef_weight_cotan[e] = cotan_weight(result_B_i, e);
            }
            auto tar_undef_weight_vert = result_B_i.vertex_property<double>("v:tar_undef_area");
            for ( auto v : result_B_i.vertices() )
            {
                tar_undef_weight_vert[v] = 0.5;
            }

            // compute per-face deformation gradients S_j of source deformation
            auto src_def_grad = result_B_i.face_property< Mat3d >("f:src_def_grad");
            for ( auto f_it : result_B_i.faces() )
            {
                // local frame M_B_0
                Eigen::MatrixXd M_B_0(3, 3);
                std::vector<Eigen::Vector3d> vertices;
                for ( auto vf_it : mesh_bs_B0_.vertices(f_it) )
                {
                    Point v = points_mesh_bs_B0_[vf_it];
                    vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
                }
                M_B_0.block<3, 1>( 0, 0 ) = vertices[1] - vertices[0];
                M_B_0.block<3, 1>( 0, 1 ) = vertices[2] - vertices[0];
                Eigen::Vector3d n_mb0 = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
                M_B_0.block<3, 1>( 0, 2 ) = n_mb0;

                // local frame M_B_i //???
                Eigen::MatrixXd M_B_i(3, 3);
                M_B_i.block<3, 2>( 0, 0 ) = solutions_X[f_it.idx()].block<3, 2>( 3*i, 0 );
                Eigen::Vector3d d1 = solutions_X[f_it.idx()].block<3, 1>( 3*i, 0 );
                Eigen::Vector3d d2 = solutions_X[f_it.idx()].block<3, 1>( 3*i, 1 );
                Eigen::Vector3d n_mbi = d1.cross(d2);
                M_B_i.block<3, 1>( 0, 2 ) = n_mbi;

                // find 3x3 matrix mapping from Mb0_i to Mbi_i
                Eigen::MatrixXd DGi_eigen = ( M_B_0 + M_B_i ) * M_B_0.inverse();
                Mat3d  DGi(DGi_eigen(0,0), DGi_eigen(0,1), DGi_eigen(0,2),
                           DGi_eigen(1,0), DGi_eigen(1,1), DGi_eigen(1,2),
                           DGi_eigen(2,0), DGi_eigen(2,1), DGi_eigen(2,2));

                // store rotation as 3x3 matrix
                src_def_grad[f_it] = DGi;
            }

            // lock closest point
            float   d, dmin(FLT_MAX);
            Surface_mesh::Vertex  vh;
            auto locked = result_B_i.vertex_property<bool>("v:locked", false);
            auto points_mesh_Ai = generic_bs_meshes_Ai_[i].vertex_property<Point>("v:point");
            for ( auto v : generic_bs_mesh_A0_.vertices() )
            {
                d = sqrnorm( points_mesh_A0_[v] - points_mesh_Ai[v] );
                if (d < dmin)
                {
                    dmin = d;
                    vh   = v;
                }
            }
            locked[vh] = true;
//            std::cerr << "(vertex: " << vh.idx() << ")... ";

            // find & count free vertices, assign indices
            auto idx = result_B_i.vertex_property<int>("v:idx");
            unsigned int N = 0; // number of free vertices
            for ( auto v : result_B_i.vertices() )
            {
                if ( !locked[v] )
                {
                    idx[v] = N++;
                }
            }

            // factorize matrix
            Eigen::SparseSPDSolver< SpMat > solver;
            SpMat A(N, N);
            std::vector<Tripl>  coeffs;
            unsigned int r = 0;
            for ( auto v : result_B_i.vertices() )
            {
                if ( !locked[v] )
                {
                    // add new row
                    double ww = 0.0;

                    auto hc     = result_B_i.halfedges(v);
                    auto hc_end = hc;
                    do
                    {
                        auto  eh =  result_B_i.edge(*hc);
                        double w = -tar_undef_weight_cotan[eh];
                        ww  -= w;
                        auto vh_to = result_B_i.to_vertex(*hc);
                        if ( !locked[vh_to] )
                        {
                            coeffs.push_back( Tripl(r, idx[vh_to], w) );
                        }
                    }
                    while (++hc != hc_end);

                    coeffs.push_back( Tripl(r, idx[v], ww) );

                    ++r;
                }
            }
            A.setFromTriplets( coeffs.begin(), coeffs.end() );
            solver.compute( A );
            if( solver.info() != Eigen::Success )
            {
                std::cerr << "Factorization failed" << std::endl;
                return;
            }

            // compute face gradients
            auto gradients = result_B_i.vertex_property<Point>("v:gradients", Point(0, 0, 0));
            for ( auto f : result_B_i.faces() )
            {
                // vertex handles & positions
                auto fvit = result_B_i.vertices(f);
                const auto va = *fvit;
                const auto vb = *(++fvit);
                const auto vc = *(++fvit);
                Point a = points_mesh_result_B_i[va];
                Point b = points_mesh_result_B_i[vb];
                Point c = points_mesh_result_B_i[vc];

                // face area
                double area = 0.5 * norm( cross( (b-a), (c-a) ) );

                // gradient of basis functions of a,b,c
                Point d   = (c-b).normalize();
                Point ga  = b + d*dot(d, (a-b)) - a;
                ga /= -sqrnorm(ga);
                d   = (c-a).normalize();
                Point gb  = a + d*dot(d, (b-a)) - b;
                gb /= -sqrnorm(gb);
                d   = (b-a).normalize();
                Point gc  = a + d*dot(d, (c-a)) - c;
                gc /= -sqrnorm(gc);

                // current source deformation gradient
                Mat3d M = src_def_grad[f];

                Mat3d G = transpose(M);
                Point gx = Point(G(0,0), G(1,0), G(2,0));
                Point gy = Point(G(0,1), G(1,1), G(2,1));
                Point gz = Point(G(0,2), G(1,2), G(2,2));

                // div: accumulate face gradient into vertex' laplacian
                gradients[va][0] -= dot(ga, gx) * area;
                gradients[va][1] -= dot(ga, gy) * area;
                gradients[va][2] -= dot(ga, gz) * area;
                gradients[vb][0] -= dot(gb, gx) * area;
                gradients[vb][1] -= dot(gb, gy) * area;
                gradients[vb][2] -= dot(gb, gz) * area;
                gradients[vc][0] -= dot(gc, gx) * area;
                gradients[vc][1] -= dot(gc, gy) * area;
                gradients[vc][2] -= dot(gc, gz) * area;
            }

            // setup right-hand side
            Eigen::MatrixXd     B(N, 3);
            unsigned int idx_r = 0;
            for ( auto v : result_B_i.vertices() )
            {
                if ( !locked[v] )
                {
                    B(idx_r, 0) = gradients[v][0] / -tar_undef_weight_vert[v];
                    B(idx_r, 1) = gradients[v][1] / -tar_undef_weight_vert[v];
                    B(idx_r, 2) = gradients[v][2] / -tar_undef_weight_vert[v];

                    double ww = 0.0;

                    auto hc     = result_B_i.halfedges(v);
                    auto hc_end = hc;
                    do
                    {
                        auto  eh = result_B_i.edge(*hc);
                        double w = -tar_undef_weight_cotan[eh];
                        ww  -= w;
                        auto vh_to = result_B_i.to_vertex(*hc);
                        if ( locked[vh_to] )
                        {
                            B(idx_r, 0) -= points_mesh_result_B_i[vh_to][0] * w;
                            B(idx_r, 1) -= points_mesh_result_B_i[vh_to][1] * w;
                            B(idx_r, 2) -= points_mesh_result_B_i[vh_to][2] * w;
                        }
                    }
                    while(++hc != hc_end);

                    ++idx_r;
                }
            }

            // solve
            Eigen::MatrixXd X = solver.solve(B);
            if( solver.info() != Eigen::Success )
            {
                std::cerr << "Solving failed" << std::endl;
                return;
            }


            // copy vertex positions
            idx_r = 0;
            for ( auto v : result_B_i.vertices() )
            {
                if ( !locked[v] )
                {
                    points_mesh_result_B_i[v][0] = X(idx_r, 0);
                    points_mesh_result_B_i[v][1] = X(idx_r, 1);
                    points_mesh_result_B_i[v][2] = X(idx_r, 2);
                    ++idx_r;
                }
            }


            // update normal vectors
            result_B_i.update_face_normals();
            result_B_i.update_vertex_normals();


            results_B_i.push_back( result_B_i );
        }


        // write individual (full) blend shapes to disk
        //if ( current_iter >= (num_iters - 1) )
        //{
            std::cout << "write individual (full) blend shapes to disk...";
            std::set<std::string>::iterator it_file = data.source_deformed_names.begin();
            for ( unsigned int i = 0; i < n; ++i )
            {
                std::string filename_result_B_i;
                filename_result_B_i.append(data.location_target_deformed);
                filename_result_B_i.append(*it_file);                
                filename_result_B_i.append( ".obj" );
                results_B_i[i].write(filename_result_B_i.c_str()); 
                ++it_file;
            }
            std::cout << "done" << std::endl;;

        //    std::cout << "break" << std::endl;;
        //    break;
        //}

        /*

        // Step B: Fix 'B_i' and solve for 'a_ij'
        std::cerr << "Step B: Fix 'B_i' and solve for 'a_ij'" << std::endl;
        const double gamma_sqrt = sqrt(gamma);
        for ( unsigned int j = 0; j < m; ++j )
        {
            // setup matrix
            SpMat A_stepB_full(3*num_vertices_ + n, n);
            Eigen::VectorXd b_stepB_full(3*num_vertices_ + n);
            std::vector< Tripl >  coeffs_stepB_A;


            // term 1
            unsigned int k = 0;
            for ( auto v : mesh_bs_B0_.vertices() )
            {
                // matrix A
                for ( unsigned int i = 0; i < n; ++i )
                {
                    // k'th point of blend shape B_i
                    Point v_k_Bi = results_B_i[i].position(v);

                    coeffs_stepB_A.push_back( Tripl( 3*k + 0, i, v_k_Bi[0] ) );
                    coeffs_stepB_A.push_back( Tripl( 3*k + 1, i, v_k_Bi[1] ) );
                    coeffs_stepB_A.push_back( Tripl( 3*k + 2, i, v_k_Bi[2] ) );
                }

                // k'th point of example shape S_j
                Point v_k_Sj = example_meshes_Sj_[j].position(v);

                // k'th point of neutral blend shape B_0
                Point v_k_B0 = mesh_bs_B0_.position(v);

                // vector b
                Point rhs_stepB_term1 = v_k_Sj - v_k_B0;
                b_stepB_full( 3*k + 0) = rhs_stepB_term1[0];
                b_stepB_full( 3*k + 1) = rhs_stepB_term1[1];
                b_stepB_full( 3*k + 2) = rhs_stepB_term1[2];

                ++k;
            }


            // term 2
            for ( unsigned int i = 0; i < n; ++i )
            {
                // matrix A
                coeffs_stepB_A.push_back( Tripl( 3*num_vertices_ + i, i, gamma_sqrt ) );

                // vector b
                b_stepB_full( 3*num_vertices_ + i) = sqrt( coeffs_aij_(i, j) * gamma_sqrt );
            }


            A_stepB_full.setFromTriplets( coeffs_stepB_A.begin(), coeffs_stepB_A.end() );
            SpMat AtA_stepB = A_stepB_full.transpose() * A_stepB_full;
            Eigen::VectorXd Atb_stepB = A_stepB_full.transpose() * b_stepB_full;
            Eigen::VectorXd x_stepB(n);


            // solve
            const bool use_quadratic_programming = true;
            if ( use_quadratic_programming )
            {
                Eigen::MatrixXd AtA_stepB_dense = AtA_stepB;

                // quadratic programming s.t. a_ij in [0 , 1]
                // matrix A
                real_2d_array alglib_A;
                alglib_A.setlength(n, n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    for (unsigned int j = 0; j < n; ++j)
                    {
                        alglib_A(i, j) = AtA_stepB_dense(i, j);
                    }
                }
                // vector b
                real_1d_array alglib_b;
                alglib_b.setlength(n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    alglib_b(i) = - Atb_stepB(i);
                }
                // vector x0 - starting point
                real_1d_array alglib_x0;
                alglib_x0.setlength(n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    alglib_x0(i) = coeffs_aij_(i, j);
                }
                // scale
                real_1d_array alglib_s;
                alglib_s.setlength(n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    alglib_s(i) = 1.0;
                }
                // lower bound
                real_1d_array alglib_bndl;
                alglib_bndl.setlength(n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    alglib_bndl(i) = 0.0;
                }
                // upper bound
                real_1d_array alglib_bndu;
                alglib_bndu.setlength(n);
                for (unsigned int i = 0; i < n; ++i)
                {
                    alglib_bndu(i) = 1.0;
                }
                // create solver, set quadratic/linear terms
                real_1d_array alglib_x;
                minqpstate alglib_state;
                minqpreport alglib_rep;
                minqpcreate(n, alglib_state);
                minqpsetquadraticterm(alglib_state, alglib_A);
                minqpsetlinearterm(alglib_state, alglib_b);
                minqpsetstartingpoint(alglib_state, alglib_x0);
                minqpsetbc(alglib_state, alglib_bndl, alglib_bndu);
                // Set scale of the parameters.
                // It is strongly recommended that you set scale of your variables.
                // Knowing their scales is essential for evaluation of stopping criteria
                // and for preconditioning of the algorithm steps.
                // You can find more information on scaling at http://www.alglib.net/optimization/scaling.php
                minqpsetscale(alglib_state, alglib_s);
                // solve problem with BLEIC-based QP solver
                // default stopping criteria are used.
                minqpsetalgobleic(alglib_state, 0.0, 0.0, 0.0, 0);
                minqpoptimize(alglib_state);
                minqpresults(alglib_state, alglib_x, alglib_rep);

                for (unsigned int i = 0; i < n; ++i)
                {
                    x_stepB(i) = alglib_x(i);
                }
            }
            else
            {
                // solve the linear system of equations
                Eigen::CholmodSupernodalLLT< SpMat >  solver_stepB;
                solver_stepB.compute(AtA_stepB);
                if ( solver_stepB.info() != Eigen::Success )
                {
                    // decomposition failed
                    std::cerr << "decomposition failed (step B)" << std::endl;
                    return;
                }
                x_stepB = solver_stepB.solve(Atb_stepB);
                if ( solver_stepB.info() != Eigen::Success )
                {
                    // solving failed
                    std::cerr << "solving failed (step B)" << std::endl;
                    return;
                }
            }


            // copy solution
            for ( unsigned int i = 0; i < n; ++i )
            {
                coeffs_aij_(i, j) = x_stepB(i);
            }
        }
        std::cerr << "coeffs_aij_.norm(): " << coeffs_aij_.norm() << std::endl;


        // decrease beta & gamma
        beta  *= beta_decr_factor;
        gamma *= gamma_decr_factor;
    }


    std::cerr << "-- a_ij ----------------------------------" << std::endl;
    std::cerr << coeffs_aij_ << std::endl;
    std::cerr << "------------------------------------------" << std::endl;
    */

    std::cout << "out 'Example_based_facial_rigging::generate_blendshapes(...)'" << std::endl;
}


//-----------------------------------------------------------------------------


void
Example_based_facial_rigging::
check_number_vertices( const Surface_mesh& mesh ) const
{
    if ( num_vertices_ != mesh.n_vertices() )
    {
        std::cerr << "ERROR: wrong number of vertices!" << std::endl;
        return;
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
