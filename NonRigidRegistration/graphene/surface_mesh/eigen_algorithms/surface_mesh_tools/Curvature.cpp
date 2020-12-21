//== INCLUDES =================================================================

#include "Curvature.h"
#include <graphene/geometry/Matrix3x3.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Curvature_analyzer::Curvature_analyzer(Surface_mesh& mesh)
  : mesh_(mesh)
{
    min_curvature_ = mesh_.add_vertex_property<Scalar>("curv:min");
    max_curvature_ = mesh_.add_vertex_property<Scalar>("curv:max");
}


//-----------------------------------------------------------------------------


Curvature_analyzer::~Curvature_analyzer()
{
    mesh_.remove_vertex_property(min_curvature_);
    mesh_.remove_vertex_property(max_curvature_);
}


//-----------------------------------------------------------------------------


void Curvature_analyzer::analyze(unsigned int smoothing_steps)
{
    Surface_mesh::Vertex_iterator vit, vend=mesh_.vertices_end();
    Surface_mesh::Edge_iterator   eit, eend=mesh_.edges_end();

    Surface_mesh::Halfedge_around_vertex_circulator vhit, vhend;
    Surface_mesh::Vertex v;


    Scalar  kmin, kmax, mean, gauss;
    Scalar  area, sum_angles;
    Scalar  weight, sum_weights;
    Point   p0, p1, p2, laplace;


    // cotan weight per edge
    Surface_mesh::Edge_property<double>  cotan = mesh_.add_edge_property<double>("curv:cotan");
    for (eit=mesh_.edges_begin(); eit!=eend; ++eit)
    {
        cotan[*eit] = cotan_weight(mesh_, *eit);
    }


    // Voronoi area per vertex
    // Laplace per vertex
    // angle sum per vertex
    // -> mean, Gauss -> min, max curvature
    for (vit=mesh_.vertices_begin(); vit!=vend; ++vit)
    {
        kmin = kmax = 0.0;

        if (!mesh_.is_isolated(*vit) && !mesh_.is_boundary(*vit))
        {
            laplace     = 0.0;
            sum_weights = 0.0;
            sum_angles  = 0.0;
            p0 = mesh_.position(*vit);

            // Voronoi area
            area = voronoi_area(mesh_, *vit);

            // Laplace & angle sum
            vhit = vhend = mesh_.halfedges(*vit);
            do
            {
                p1 = mesh_.position(mesh_.to_vertex(*vhit));
                p2 = mesh_.position(mesh_.to_vertex(mesh_.ccw_rotated_halfedge(*vhit)));

                weight       = cotan[mesh_.edge(*vhit)];
                sum_weights += weight;
                laplace     += weight * p1;

                p1 -= p0;  p1.normalize();
                p2 -= p0;  p2.normalize();
                sum_angles += acos( clamp_cos( dot(p1, p2) ) );
            }
            while (++vhit != vhend);
            laplace -= sum_weights * mesh_.position(*vit);
            laplace /= Scalar(2.0) * area;

            mean = Scalar(0.5) * norm(laplace);
            gauss = (2.0*M_PI - sum_angles) / area;

            const Scalar s = sqrt(std::max(Scalar(0.0), mean*mean-gauss));
            kmin = mean - s;
            kmax = mean + s;
        }

        min_curvature_[*vit] = kmin;
        max_curvature_[*vit] = kmax;
    }



    // boundary vertices: interpolate from interior neighbors
    for (vit=mesh_.vertices_begin(); vit!=vend; ++vit)
    {
        if (mesh_.is_boundary(*vit))
        {
            kmin = kmax = sum_weights = 0.0;

            vhit = vhend = mesh_.halfedges(*vit);
            if (vhit) do
                      {
                          v = mesh_.to_vertex(*vhit);
                          if (!mesh_.is_boundary(v))
                          {
                              weight = cotan[mesh_.edge(*vhit)];
                              sum_weights += weight;
                              kmin += weight * min_curvature_[v];
                              kmax += weight * max_curvature_[v];
                          }
                      }
                while (++vhit != vhend);

            if (sum_weights)
            {
                kmin /= sum_weights;
                kmax /= sum_weights;
            }

            min_curvature_[*vit] = kmin;
            max_curvature_[*vit] = kmax;
        }
    }


    // clean-up properties
    mesh_.remove_edge_property(cotan);


    // smooth curvature values
    smooth_curvatures(smoothing_steps);
}


//-----------------------------------------------------------------------------


void Curvature_analyzer::analyze_tensor(unsigned int smoothing_steps,
                                        bool two_ring_neighborhood)
{
    Surface_mesh::Vertex_property<double> area   = mesh_.add_vertex_property<double>("curv:area", 0.0);
    Surface_mesh::Face_property<Vec3d>    normal = mesh_.add_face_property<Vec3d>("curv:normal");
    Surface_mesh::Edge_property<Vec3d>    evec   = mesh_.add_edge_property<Vec3d>("curv:evec", Vec3d(0,0,0));
    Surface_mesh::Edge_property<double>   angle  = mesh_.add_edge_property<double>("curv:angle", 0.0);

    Surface_mesh::Vertex_iterator vit, vend=mesh_.vertices_end();
    Surface_mesh::Edge_iterator   eit, eend=mesh_.edges_end();
    Surface_mesh::Face_iterator   fit, fend=mesh_.faces_end();

    Surface_mesh::Vertex_around_vertex_circulator    vvit, vvend;
    Surface_mesh::Vertex_around_face_circulator      vfit;
    Surface_mesh::Halfedge_around_vertex_circulator  hvit, hvend;

    Surface_mesh::Vertex    v;
    Surface_mesh::Halfedge  h0, h1;
    Surface_mesh::Edge      ee;
    Surface_mesh::Face      f0, f1;
    Vec3d                   p0, p1, n0, n1, e;
    double                  l, A, beta, a1, a2, a3;
    Mat3d                   tensor;

    double  eval1, eval2, eval3, kmin, kmax;
    Vec3d   evec1, evec2, evec3;

    std::vector<Surface_mesh::Vertex>  neighborhood;
    std::vector<Surface_mesh::Vertex>::const_iterator nit, nend;
    neighborhood.reserve(15);



    // precompute Voronoi area per vertex
    for (vit=mesh_.vertices_begin(); vit!=vend; ++vit)
    {
        area[*vit] = voronoi_area(mesh_, *vit);
    }


    // precompute face normals
    for (fit=mesh_.faces_begin(); fit!=fend; ++fit)
    {
        normal[*fit] = (Vec3d) mesh_.compute_face_normal(*fit);
    }


    // precompute dihedral_angle*edge_length*edge per edge
    for (eit=mesh_.edges_begin(); eit!=eend; ++eit)
    {
        h0 = mesh_.halfedge(*eit, 0);
        h1 = mesh_.halfedge(*eit, 1);
        f0 = mesh_.face(h0);
        f1 = mesh_.face(h1);
        if (f0.is_valid() && f1.is_valid())
        {
            n0 = normal[f0];
            n1 = normal[f1];
            e  = mesh_.position(mesh_.to_vertex(h0));
            e -= mesh_.position(mesh_.to_vertex(h1));
            l  = norm(e);
            e /= l;
            l *= 0.5; // only consider half of the edge (matchig Voronoi area)
            angle[*eit] = atan2(dot(cross(n0,n1), e), dot(n0,n1));
            evec[*eit]  = sqrt(l) * e;
        }
    }


    // compute curvature tensor for each vertex
    for (vit=mesh_.vertices_begin(); vit!=vend; ++vit)
    {
        kmin = 0.0;
        kmax = 0.0;


        if (!mesh_.is_isolated(*vit))
        {
            // one-ring or two-ring neighborhood?
            neighborhood.clear();
            neighborhood.push_back(*vit);
            if (two_ring_neighborhood)
            {
                vvit = vvend = mesh_.vertices(*vit);
                do
                {
                    neighborhood.push_back(*vvit);
                }
                while (++vvit != vvend);
            }


            A = 0.0;
            tensor = Mat3d::zero();


            // compute tensor over vertex neighborhood stored in vertices
            for (nit=neighborhood.begin(), nend=neighborhood.end(); nit!=nend; ++nit)
            {
                // accumulate tensor from dihedral angles around vertices
                hvit = hvend = mesh_.halfedges(*nit);
                do
                {
                    ee = mesh_.edge(*hvit);
                    e = evec[ee];
                    beta = angle[ee];
                    for (int i=0; i<3; ++i)
                        for (int j=0; j<3; ++j)
                            tensor(i,j) += beta * e[i]*e[j];
                }
                while (++hvit != hvend);

                // accumulate area
                A += area[*nit];
            }

            // normalize tensor by accumulated
            tensor /= A;


            // Eigen-decomposition
            bool ok = symmetric_eigendecomposition(tensor, eval1, eval2, eval3, evec1, evec2, evec3);
            if (ok)
            {
                // curvature values:
                //   normal vector -> eval with smallest absolute value
                //   evals are sorted in decreasing order
                a1 = fabs(eval1);
                a2 = fabs(eval2);
                a3 = fabs(eval3);
                if (a1 < a2)
                {
                    if (a1 < a3)
                    {
                        // e1 is normal
                        kmax = eval2;
                        kmin = eval3;
                    }
                    else
                    {
                        // e3 is normal
                        kmax = eval1;
                        kmin = eval2;
                    }
                }
                else
                {
                    if (a2 < a3)
                    {
                        // e2 is normal
                        kmax = eval1;
                        kmin = eval3;
                    }
                    else
                    {
                        // e3 is normal
                        kmax = eval1;
                        kmin = eval2;
                    }
                }
            }
        }

        assert(kmin <= kmax);

        min_curvature_[*vit] = kmin;
        max_curvature_[*vit] = kmax;
    }


    // clean-up properties
    mesh_.remove_vertex_property(area);
    mesh_.remove_edge_property(evec);
    mesh_.remove_edge_property(angle);
    mesh_.remove_face_property(normal);


    // smooth curvature values
    smooth_curvatures(smoothing_steps);
}


//-----------------------------------------------------------------------------


void Curvature_analyzer::smooth_curvatures(unsigned int _iterations)
{
    Surface_mesh::Vertex_iterator  vit, vend=mesh_.vertices_end();
    Surface_mesh::Edge_iterator    eit, eend=mesh_.edges_end();
    Surface_mesh::Halfedge_around_vertex_circulator vhit, vhend;

    Surface_mesh::Vertex v;
    Scalar  kmin, kmax;
    Scalar  weight, sum_weights;


    // properties
    Surface_mesh::Vertex_property<bool> vfeature = mesh_.get_vertex_property<bool>("v:feature");
    Surface_mesh::Edge_property<double>  cotan = mesh_.add_edge_property<double>("curv:cotan");


    // cotan weight per edge
    for (eit=mesh_.edges_begin(); eit!=eend; ++eit)
    {
        cotan[*eit] = cotan_weight(mesh_, *eit);
    }


    for (unsigned int i=0; i<_iterations; ++i)
    {
        for (vit=mesh_.vertices_begin(); vit!=vend; ++vit)
        {
            // don't smooth feature vertices
            if (vfeature && vfeature[*vit])
                continue;

            kmin = kmax = sum_weights = 0.0;

            vhit = vhend = mesh_.halfedges(*vit);
            if (vhit) do
                      {
                          v = mesh_.to_vertex(*vhit);

                          // don't consider feature vertices (high curvature)
                          if (vfeature && vfeature[v])
                              continue;

                          weight = std::max(0.0, cotan_weight(mesh_, mesh_.edge(*vhit)));
                          sum_weights += weight;
                          kmin += weight * min_curvature_[v];
                          kmax += weight * max_curvature_[v];
                      }
                while (++vhit != vhend);

            if (sum_weights)
            {
                min_curvature_[*vit] = kmin / sum_weights;
                max_curvature_[*vit] = kmax / sum_weights;
            }
        }
    }


    // remove property
    mesh_.remove_edge_property(cotan);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
