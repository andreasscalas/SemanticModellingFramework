//== INCLUDES =================================================================

#include "diffgeo.h"
#include <limits>
#include <cmath>
#include <float.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


Scalar triangle_area(const Point& p0, const Point& p1, const Point& p2)
{
    return Scalar(0.5) * norm(cross(p1-p0, p2-p0));
}


//-----------------------------------------------------------------------------


Scalar triangle_area(const Surface_mesh& mesh, Surface_mesh::Face f)
{
    assert(mesh.valence(f) == 3);

    Surface_mesh::Vertex_around_face_circulator vfit = mesh.vertices(f);
    const Point& p0 = mesh.position(*vfit);
    ++vfit;
    const Point& p1 = mesh.position(*vfit);
    ++vfit;
    const Point& p2 = mesh.position(*vfit);

    return triangle_area(p0, p1, p2);
}


//-----------------------------------------------------------------------------


double cotan_weight(const Surface_mesh& mesh, Surface_mesh::Edge e)
{
    double weight  = 0.0;

    const Surface_mesh::Halfedge  h0 = mesh.halfedge(e, 0);
    const Surface_mesh::Halfedge  h1 = mesh.halfedge(e, 1);

    const Vec3d p0 = (Vec3d) mesh.position(mesh.to_vertex(h0));
    const Vec3d p1 = (Vec3d) mesh.position(mesh.to_vertex(h1));

    if (!mesh.is_boundary(h0))
    {
        const Vec3d p2 = (Vec3d) mesh.position(mesh.to_vertex(mesh.next_halfedge(h0)));
        const Vec3d d0 = p0 - p2;
        const Vec3d d1 = p1 - p2;
        const double area = norm(cross(d0,d1));
        if (area > std::numeric_limits<double>::min())
        {
            const double cot  = dot(d0,d1) / area;
            weight += clamp_cot(cot);
        }
    }

    if (!mesh.is_boundary(h1))
    {
        const Vec3d p2 = (Vec3d) mesh.position(mesh.to_vertex(mesh.next_halfedge(h1)));
        const Vec3d d0 = p0 - p2;
        const Vec3d d1 = p1 - p2;
        const double area = norm(cross(d0,d1));
        if (area > std::numeric_limits<double>::min())
        {
            const double cot  = dot(d0,d1) / area;
            weight += clamp_cot(cot);
        }
    }

#ifdef _WIN32
    assert(_finite(weight));  // checks for NaN and INF
#else
    assert(!std::isnan(weight));
    assert(!std::isinf(weight));
#endif

    return weight;
}


//-----------------------------------------------------------------------------


double voronoi_area(const Surface_mesh& mesh, Surface_mesh::Vertex v)
{
    double area(0.0);

    if (!mesh.is_isolated(v))
    {
        Surface_mesh::Halfedge h0, h1, h2;
        Vec3d   P, Q, R, PQ, QR, PR;
        double  dotP, dotQ, dotR, tri_area;
        double  cotQ, cotR;

        Surface_mesh::Halfedge_around_vertex_circulator vhit=mesh.halfedges(v), vhend=vhit;
        do
        {
            h0 = *vhit;
            h1 = mesh.next_halfedge(h0);
            h2 = mesh.next_halfedge(h1);

            if (mesh.is_boundary(h0))
                continue;

            // three vertex positions
            P = (Vec3d) mesh.position(mesh.to_vertex(h2));
            Q = (Vec3d) mesh.position(mesh.to_vertex(h0));
            R = (Vec3d) mesh.position(mesh.to_vertex(h1));

            // edge vectors
            (PQ = Q) -= P;
            (QR = R) -= Q;
            (PR = R) -= P;

            // compute and check triangle area
            tri_area = norm(cross(PQ, PR));
            if (tri_area <= std::numeric_limits<double>::min())
                continue;

            // dot products for each corner (of its two emanating edge vectors)
            dotP =  dot(PQ, PR);
            dotQ = -dot(QR, PQ);
            dotR =  dot(QR, PR);


            // angle at P is obtuse
            if (dotP < 0.0)
            {
                area += 0.25 * tri_area;
            }

            // angle at Q or R obtuse
            else if (dotQ < 0.0 || dotR < 0.0)
            {
                area += 0.125 * tri_area;
            }

            // no obtuse angles
            else
            {
                // cot(angle) = cos(angle)/sin(angle) = dot(A,B)/norm(cross(A,B))
                cotQ = dotQ / tri_area;
                cotR = dotR / tri_area;

                // clamp cot(angle) by clamping angle to [1,179]
                area += 0.125 * (sqrnorm(PR) * clamp_cot(cotQ) +
                                 sqrnorm(PQ) * clamp_cot(cotR) );
            }
        }
        while (++vhit != vhend);
    }

#ifdef _WIN32
    assert(_finite(area));  // checks for NaN and Inf
#else
    assert(!std::isnan(area));
    assert(!std::isinf(area));
#endif

    return area;
}


//-----------------------------------------------------------------------------


double voronoi_area_barycentric(const Surface_mesh& mesh, Surface_mesh::Vertex v)
{
    double area(0.0);

    if (!mesh.is_isolated(v))
    {
        const Point P = mesh.position(v);
        Surface_mesh::Halfedge h0, h1;
        Point   Q, R, PQ, PR;


        Surface_mesh::Halfedge_around_vertex_circulator vhit=mesh.halfedges(v), vhend=vhit;
        do
        {
            if (mesh.is_boundary(*vhit))
                continue;

            h0 = *vhit;
            h1 = mesh.next_halfedge(h0);

            PQ = mesh.position(mesh.to_vertex(h0));
            PQ -= P;

            PR = mesh.position(mesh.to_vertex(h1));
            PR -= P;

            area += norm(cross(PQ, PR)) / 3.0;
        }
        while (++vhit != vhend);
    }

    return area;
}


//-----------------------------------------------------------------------------


Point laplace(const Surface_mesh& mesh, Surface_mesh::Vertex v)
{
    Point   laplace(0.0, 0.0, 0.0);

    if (!mesh.is_isolated(v))
    {
        Scalar  weight, sum_weights(0.0);

        Surface_mesh::Halfedge_around_vertex_circulator vhit=mesh.halfedges(v), vhend=vhit;
        do
        {
            weight       = cotan_weight(mesh, mesh.edge(*vhit));
            sum_weights += weight;
            laplace     += weight * mesh.position(mesh.to_vertex(*vhit));
        }
        while (++vhit != vhend);

        laplace -= sum_weights * mesh.position(v);
        laplace /= Scalar(2.0) * voronoi_area(mesh, v);
    }

    return laplace;
}


//-----------------------------------------------------------------------------


Scalar angle_sum(const Surface_mesh& mesh, Surface_mesh::Vertex v)
{
    Scalar angles(0.0);

    if (!mesh.is_boundary(v))
    {
        const Point& p0 = mesh.position(v);

        Surface_mesh::Halfedge_around_vertex_circulator vhit=mesh.halfedges(v), vhend=vhit;
        do
        {
            const Point& p1 = mesh.position(mesh.to_vertex(*vhit));
            const Point& p2 = mesh.position(mesh.to_vertex(mesh.ccw_rotated_halfedge(*vhit)));

            const Point p01 = (p1-p0).normalize();
            const Point p02 = (p2-p0).normalize();

            Scalar cos_angle = clamp_cos(dot(p01, p02));

            angles += acos(cos_angle);
        }
        while (++vhit != vhend);
    }

    return angles;
}


//-----------------------------------------------------------------------------


Vertex_curvature vertex_curvature(const Surface_mesh& mesh, Surface_mesh::Vertex v)
{
    Vertex_curvature c;

    const Scalar area = voronoi_area(mesh, v);
    if (area > std::numeric_limits<Scalar>::min())
    {
        c.mean = Scalar(0.5) * norm(laplace(mesh, v));
        c.gauss = (2.0*M_PI - angle_sum(mesh, v)) / voronoi_area(mesh, v);

        const Scalar s = sqrt(std::max(Scalar(0.0), c.mean*c.mean-c.gauss));
        c.min = c.mean - s;
        c.max = c.mean + s;

#ifdef _WIN32
        assert(_finite(c.mean));  // checks for NaN and Inf
        assert(_finite(c.gauss));  // checks for NaN and Inf
#else
        assert(!std::isnan(c.mean));
        assert(!std::isnan(c.gauss));
        assert(!std::isinf(c.mean));
        assert(!std::isinf(c.gauss));
#endif
        assert(c.min <= c.mean);
        assert(c.mean <= c.max);
    }

    return c;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
