//== INCLUDES ===================================================================


#include "Deformation_transfer_sumner.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================


Deformation_transfer_sumner::
Deformation_transfer_sumner( Surface_mesh& mesh_target_undeformed )
    : mesh_result_( mesh_target_undeformed )
{
}


//-----------------------------------------------------------------------------


Deformation_transfer_sumner::
~Deformation_transfer_sumner()
{
}


//-----------------------------------------------------------------------------


bool
Deformation_transfer_sumner::
deform_target( const std::vector<Point> &mesh_source_undeformed,
               const std::vector<Point> &mesh_source_deformed )
{
    if ( ! mesh_result_.is_triangle_mesh() )
    {
        return false;
    }


    // check dimensions
    if ( mesh_result_.n_vertices() == 0                             ||
         mesh_result_.n_vertices() != mesh_source_undeformed.size() ||
         mesh_result_.n_vertices() != mesh_source_deformed.size() )
    {
        return false;
    }


    // store original undeformed points of target mesh
    auto points_mesh_result            = mesh_result_.vertex_property<Point>("v:point");

    auto points_target_mesh_undeformed = mesh_result_.get_vertex_property<Point>("v:tar_undef_points");
    if ( !points_target_mesh_undeformed )
    {
        points_target_mesh_undeformed = mesh_result_.vertex_property<Point>("v:tar_undef_points");

        for ( auto v: mesh_result_.vertices() )
        {
            points_target_mesh_undeformed[v] = points_mesh_result[v];
        }
    }


    unsigned int num_vertices = mesh_result_.n_vertices();
    unsigned int num_tris     = mesh_result_.n_faces();


    // accumulate the vertices for the undeformed source mesh
    Eigen::Matrix<double, Eigen::Dynamic, 3> V_src_undef(3 * num_tris, 3);
    for ( auto f_it : mesh_result_.faces() )
    {
        std::vector<Eigen::Vector3d> vertices;
        for ( auto vf_it : mesh_result_.vertices(f_it) )
        {
            Point v = mesh_source_undeformed[vf_it.idx()];
            vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
        }

        Eigen::Vector3d v_cross = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        Eigen::Vector3d v3 = vertices[0] + v_cross / v_cross.norm();

        V_src_undef.block<3, 1>( 3*f_it.idx(), 0 ) = vertices[1] - vertices[0];
        V_src_undef.block<3, 1>( 3*f_it.idx(), 1 ) = vertices[2] - vertices[0];;
        V_src_undef.block<3, 1>( 3*f_it.idx(), 2 ) =          v3 - vertices[0];;
    }


    // accumulate the template vertices for the expression
    Eigen::Matrix<double, Eigen::Dynamic, 3> V_src_def(3 * num_tris, 3);
    for ( auto f_it : mesh_result_.faces() )
    {
        std::vector<Eigen::Vector3d> vertices;
        for ( auto vf_it : mesh_result_.vertices(f_it) )
        {
            Point v = mesh_source_deformed[vf_it.idx()];
            vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
        }

        Eigen::Vector3d v_cross = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        Eigen::Vector3d v3 = vertices[0] + v_cross / v_cross.norm();

        V_src_def.block<3, 1>( 3*f_it.idx(), 0 ) = vertices[1] - vertices[0];
        V_src_def.block<3, 1>( 3*f_it.idx(), 1 ) = vertices[2] - vertices[0];;
        V_src_def.block<3, 1>( 3*f_it.idx(), 2 ) =          v3 - vertices[0];;
    }


    // compute deformation gradients of source deformation and store them in a big vector
    Eigen::VectorXd b(9 * num_tris);
    for (size_t f_i = 0; f_i < num_tris; ++f_i)
    {
        Eigen::Matrix3d b_block = V_src_def.block<3, 3> (3 * f_i, 0) *
                                 (V_src_undef.block<3, 3>(3*f_i, 0)).inverse();
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                b(9 * f_i + 3 * i + j) = b_block(i, j);
            }
        }
    }


    // build the matrix
    Eigen::SparseMatrix<double> J(9 * num_tris, 3 * (num_vertices + num_tris));
    std::vector< Eigen::Triplet<double> > triplets;
    Eigen::VectorXd b_reg( Eigen::VectorXd::Zero(3 * (num_vertices + num_tris)) );
    for ( auto f_it : mesh_result_.faces() )
    {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<int> v_ids;
        for ( auto vf_it : mesh_result_.vertices(f_it) )
        {
            Point v = points_target_mesh_undeformed[vf_it];
            vertices.push_back( Eigen::Vector3d(v[0], v[1], v[2]) );
            v_ids.push_back( vf_it.idx() );
        }

        Eigen::Vector3d v_cross = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        Eigen::Vector3d v3 = vertices[0] + v_cross / v_cross.norm ();

        Eigen::Matrix<double, 3, 3> V;
        V.col(0) = vertices[1] - vertices[0];
        V.col(1) = vertices[2] - vertices[0];
        V.col(2) =          v3 - vertices[0];
        Eigen::Matrix<double, 3, 3> Vinv = V.inverse();
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                triplets.push_back(Eigen::Triplet<double> (9 * f_it.idx() + 3 * i + j,
                                                           3 * v_ids[0] + i,
                                                           -Vinv(0, j) -Vinv(1, j) -Vinv(2, j) ));

                triplets.push_back(Eigen::Triplet<double> (9 * f_it.idx() + 3 * i + j,
                                                           3 * v_ids[1] + i,
                                                           Vinv(0, j) ));

                triplets.push_back(Eigen::Triplet<double> (9 * f_it.idx() + 3 * i + j,
                                                           3 * v_ids[2] + i,
                                                           Vinv(1, j) ));

                triplets.push_back(Eigen::Triplet<double> (9 * f_it.idx() + 3 * i + j,
                                                           3 * num_vertices + 3 * f_it.idx() + i,
                                                           Vinv(2, j) ));
            }
        }

        // regularization
        for (size_t i = 0; i < 3; ++i)
        {
            b_reg( 3*num_vertices + 3*f_it.idx() + i ) = v3(i);
        }
    }
    J.setFromTriplets( triplets.begin(), triplets.end() );


    // regularization
    Eigen::SparseMatrix<double> Id(3 * (num_vertices + num_tris), 3 * (num_vertices + num_tris));
    Id.setIdentity();
    for ( auto v_it : mesh_result_.vertices() )
    {
        Point p = points_target_mesh_undeformed[v_it];
        for (size_t i = 0; i < 3; ++i)
        {
            b_reg( 3 * v_it.idx() + i ) = p[i];
        }
    }


    // solve linear system
    std::cerr << "Solving sparse linear system ..." << std::endl;
    Eigen::SparseMatrix<double> JtJ = J.transpose()*J + Id;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(JtJ);
    std::cerr << "Decomposing ..." << std::endl;
    if (solver.info () != Eigen::Success)
    {
        std::cerr << "Solver decomposition failed." << std::endl;
        return false;
    }
    std::cerr << "Solving ..." << std::endl;
    Eigen::VectorXd sol = solver.solve( J.transpose()*b + b_reg );


    // put the solution back
    for ( auto v_it : mesh_result_.vertices() )
    {
        points_mesh_result[v_it] = Point( sol (3 * v_it.idx() + 0),
                                          sol (3 * v_it.idx() + 1),
                                          sol (3 * v_it.idx() + 2) );
    }
    mesh_result_.update_face_normals ();
    mesh_result_.update_vertex_normals ();


    return true;
}


//-----------------------------------------------------------------------------


Surface_mesh
Deformation_transfer_sumner::
get_deformed_target()
{
    return mesh_result_;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
