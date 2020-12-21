//== INCLUDES ===================================================================


#include "Deformation_transfer_botsch.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/diffgeo.h>
#include <graphene/geometry/Matrix3x3.h>

#include <cfloat>


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================


Deformation_transfer_botsch::
Deformation_transfer_botsch( ):
   Deformation_transfer()
{ 
}


//-----------------------------------------------------------------------------


Deformation_transfer_botsch::
~Deformation_transfer_botsch()
{  
}

//-----------------------------------------------------------------------------


void 
Deformation_transfer_botsch::
prefactorize(Surface_mesh& mesh_target_undeformed, unsigned int& mat_size)
{
    if (!mesh_target_undeformed.is_triangle_mesh())
    {
        std::cerr << "Deformation Transfer failed! Target is not a triangle mesh." << std::endl;
        return;
    }

    // compute edge and vertex weights
    auto tar_undef_weight_cotan = mesh_target_undeformed.get_edge_property<double>("e:tar_undef_cotan");
    if (!tar_undef_weight_cotan)
    {
        tar_undef_weight_cotan = mesh_target_undeformed.edge_property<double>("e:tar_undef_cotan");

        for (auto e : mesh_target_undeformed.edges())
        {
            tar_undef_weight_cotan[e] = cotan_weight(mesh_target_undeformed, e);
        }
    }

    // find & count free vertices, assign indices
    auto locked = mesh_target_undeformed.get_vertex_property<bool>("v:locked");
    if (!locked)
    {
        std::cerr << "Deformaion Transfer failed! No fix vertices defined" << std::endl;
        return;
    }
    auto idx = mesh_target_undeformed.vertex_property<int>("v:idx");
    unsigned int N = 0; // number of free vertices
    for (auto v : mesh_target_undeformed.vertices())
    {
        if (!locked[v])
        {
            idx[v] = N++;
        }
    }
    mat_size = N;


    // factorize matrix
    solver_ = new Eigen::SparseSPDSolver< SpMat >;
    A_ = new SpMat(N, N);
    std::vector<Tripl>  coeffs;
    unsigned int r = 0;
    for (auto v : mesh_target_undeformed.vertices())
    {
        if (!locked[v])
        {
            // add new row
            double ww = 0.0;

            auto hc = mesh_target_undeformed.halfedges(v);
            auto hc_end = hc;
            do
            {
                auto  eh = mesh_target_undeformed.edge(*hc);
                double w = -tar_undef_weight_cotan[eh];
                ww -= w;
                auto vh_to = mesh_target_undeformed.to_vertex(*hc);
                if (!locked[vh_to])
                {
                    coeffs.push_back(Tripl(r, idx[vh_to], w));
                }
            }
            while (++hc != hc_end);

            coeffs.push_back(Tripl(r, idx[v], ww));

            ++r;
        }
    }
    A_->setFromTriplets(coeffs.begin(), coeffs.end());
    solver_->compute(*A_);
    if (solver_->info() != Eigen::Success)
    {
        std::cerr << "Deformaion Transfer failed!  Factorization failed." << std::endl;
        factorization_state=false;
    }
    else
    {
        factorization_state = true;
    }
}


//-----------------------------------------------------------------------------


bool
Deformation_transfer_botsch::
create_deformed_target(Surface_mesh& mesh_source_undeformed,
               Surface_mesh& mesh_source_deformed,
               Surface_mesh& mesh_target_undeformed,
               Surface_mesh& mesh_target_deformed,
               unsigned int mat_size)
{
    // check factorization
    if (factorization_state == false)
    {
        return false;
    }

    const unsigned int tar_n_vertices = mesh_target_undeformed.n_vertices();
    // check dimensions
    if ( tar_n_vertices  == 0 ||
         tar_n_vertices != mesh_source_undeformed.n_vertices() ||
         tar_n_vertices != mesh_source_deformed.n_vertices())
    {
        std::cerr << "Deformaion Transfer failed! Meshes have different amount of vertices." << std::endl;
        return false;
    }


    // compute per-face deformation gradients S_j of source deformation
    auto points_source_undeformed = mesh_source_undeformed.vertex_property<Point>("v:point");
    auto points_source_deformed   = mesh_source_deformed.vertex_property<Point>("v:point");
    auto src_def_grad             = mesh_target_deformed.face_property< Mat3d >("f:src_def_grad");

    for (auto f : mesh_target_deformed.faces())
    {
        // collect points
        auto fvit = mesh_target_deformed.vertices(f);
        const Point& s1 = points_source_undeformed[(*fvit)];
        const Point& d1 = points_source_deformed[(*fvit)];
        ++fvit;
        const Point& s2 = points_source_undeformed[(*fvit)];
        const Point& d2 = points_source_deformed[(*fvit)];
        ++fvit;
        const Point& s3 = points_source_undeformed[(*fvit)];
        const Point& d3 = points_source_deformed[(*fvit)];

        // build frames
        Point a1 = s2-s1;
        Point a2 = s3-s1;
        Point a3 = cross(a1, a2).normalize();
        Point b1 = d2-d1;
        Point b2 = d3-d1;
        Point b3 = cross(b1, b2).normalize();

        // find 3x3 matrix mapping from a_i to b_i
        Mat3d Ma(a1[0], a2[0], a3[0],
                 a1[1], a2[1], a3[1],
                 a1[2], a2[2], a3[2]);
        Mat3d Mb(b1[0], b2[0], b3[0],
                 b1[1], b2[1], b3[1],
                 b1[2], b2[2], b3[2]);
        Mat3d M = Mb * inverse(Ma);

        // store rotation as 3x3 matrix
        src_def_grad[f] = M;
    }

    // get positions and weights
    auto points_target_mesh_undeformed = mesh_target_undeformed.vertex_property<Point>("v:point");
    auto tar_undef_weight_vert = mesh_target_undeformed.get_vertex_property<double>("v:tar_undef_area");
    if (!tar_undef_weight_vert)
    {
       tar_undef_weight_vert = mesh_target_undeformed.vertex_property<double>("v:tar_undef_area");

       for (auto v : mesh_target_undeformed.vertices())
       {
          tar_undef_weight_vert[v] = 0.5;
       }
    }
    // compute edge and vertex weights
    auto tar_undef_weight_cotan = mesh_target_undeformed.get_edge_property<double>("e:tar_undef_cotan");
    if (!tar_undef_weight_cotan)
    {
        tar_undef_weight_cotan = mesh_target_undeformed.edge_property<double>("e:tar_undef_cotan");

        for (auto e : mesh_target_undeformed.edges())
        {
            tar_undef_weight_cotan[e] = cotan_weight(mesh_target_undeformed, e);
        }
    }

    // compute face gradients for right hand side
    auto gradients = mesh_target_undeformed.vertex_property<Point>("v:gradients", Point(0, 0, 0));
    for (auto f : mesh_target_undeformed.faces())
    {
        // vertex handles & positions
        auto fvit = mesh_target_undeformed.vertices(f);
        const auto va = *fvit;
        const auto vb = *(++fvit);
        const auto vc = *(++fvit);
        Point a = points_target_mesh_undeformed[va];
        Point b = points_target_mesh_undeformed[vb];
        Point c = points_target_mesh_undeformed[vc];

        // face area
        double area = 0.5 * norm(cross((b - a), (c - a)));

        // gradient of basis functions of a
        Point d = (c - b).normalize();
        Point ga = b + d*dot(d, (a - b)) - a;
        ga /= -sqrnorm(ga);
        // gradient of basis functions of b
        d = (c - a).normalize();
        Point gb = a + d*dot(d, (b - a)) - b;
        gb /= -sqrnorm(gb);
        // gradient of basis functions of c
        d = (b - a).normalize();
        Point gc = a + d*dot(d, (c - a)) - c;
        gc /= -sqrnorm(gc);

        // current source deformation gradient
        Mat3d M = src_def_grad[f];

        Mat3d G = transpose(M);
        Point gx = Point(G(0, 0), G(1, 0), G(2, 0)); // x components of all three gradients
        Point gy = Point(G(0, 1), G(1, 1), G(2, 1)); // y components of all three gradients
        Point gz = Point(G(0, 2), G(1, 2), G(2, 2)); // z components of all three gradients

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

    // find & count free vertices, assign indices
    auto locked = mesh_target_undeformed.get_vertex_property<bool>("v:locked");
    auto idx    = mesh_target_undeformed.get_vertex_property<int>("v:idx");
    if (!locked || !idx)
    {
       std::cerr << "Deformaion Transfer failed! No fix vertices defined" << std::endl;
       return false;
    }

    // setup right-hand side
    Eigen::MatrixXd     B(mat_size, 3);
    unsigned int i = 0;
    for (auto v : mesh_target_undeformed.vertices())
    {
        if ( !locked[v] )
        {
            B(i, 0) = gradients[v][0] / -tar_undef_weight_vert[v];
            B(i, 1) = gradients[v][1] / -tar_undef_weight_vert[v];
            B(i, 2) = gradients[v][2] / -tar_undef_weight_vert[v];

            double ww = 0.0;

            auto hc = mesh_target_undeformed.halfedges(v);
            auto hc_end = hc;
            do
            {
               auto  eh = mesh_target_undeformed.edge(*hc);
                double w = -tar_undef_weight_cotan[eh];
                ww  -= w;
                auto vh_to = mesh_target_undeformed.to_vertex(*hc);
                if ( locked[vh_to] )
                {
                    B(i, 0) -= points_target_mesh_undeformed[vh_to][0] * w;
                    B(i, 1) -= points_target_mesh_undeformed[vh_to][1] * w;
                    B(i, 2) -= points_target_mesh_undeformed[vh_to][2] * w;
                }
            }
            while(++hc != hc_end);

            ++i;
        }
    }


    // solve
    Eigen::MatrixXd X(mat_size, 3);
    X = solver_->solve(B);
    if( solver_->info() != Eigen::Success )
    {
        std::cerr << "Solving failed" << std::endl;
        return false;
    }


    // copy vertex positions
    auto points_target_mesh_deformed = mesh_target_deformed.vertex_property<Point>("v:point");
    i = 0;
    for (auto v : mesh_target_deformed.vertices())
    {
        if ( !locked[v] )
        {
           points_target_mesh_deformed[v][0] = X(i, 0);
           points_target_mesh_deformed[v][1] = X(i, 1);
           points_target_mesh_deformed[v][2] = X(i, 2);
            ++i;
        }
    }


    // update normal vectors
    mesh_target_deformed.update_face_normals();
    mesh_target_deformed.update_vertex_normals();


    // clean up
    mesh_target_deformed.remove_face_property(src_def_grad);
    //mesh_result_.remove_vertex_property( locked );
    //mesh_result_.remove_vertex_property( idx );
    mesh_target_undeformed.remove_vertex_property(gradients);

    return true;
}


//-----------------------------------------------------------------------------


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
