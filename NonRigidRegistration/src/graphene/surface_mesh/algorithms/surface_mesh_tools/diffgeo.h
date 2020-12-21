//=============================================================================

#ifndef DIFF_GEO_H
#define DIFF_GEO_H


//== INCLUDES =================================================================

#include <graphene/types.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================

// clamp cotangent and cosince values as if angles are in [1, 179]

inline double clamp_cot(const double v)
{
    const double bound = 19.1; // 3 degrees
    return (v < -bound ? -bound : (v > bound ? bound : v));
}

inline double clamp_cos(const double v)
{
    const double bound = 0.9986; // 3 degrees
    return (v < -bound ? -bound : (v > bound ? bound : v));
}

// compute angle between two (un-normalized) vectors
inline Scalar angle(const Point& v0, const Point& v1)
{
    return atan2(norm(cross(v0,v1)), dot(v0,v1));
}

// compute sine of angle between two (un-normalized) vectors
inline Scalar sin(const Point& v0, const Point& v1)
{
    return norm(cross(v0,v1)) / (norm(v0) * norm(v1));
}

// compute cosine of angle between two (un-normalized) vectors
inline Scalar cos(const Point& v0, const Point& v1)
{
    return dot(v0,v1) / (norm(v0) * norm(v1));
}

// compute cotangent of angle between two (un-normalized) vectors
inline Scalar cotan(const Point& v0, const Point& v1)
{
    return clamp_cot(dot(v0,v1) / norm(cross(v0,v1)));
}

/// compute area of a triangle given by three points
Scalar triangle_area(const Point& p0, const Point& p1, const Point& p2);

/// compute area of triangle f
Scalar triangle_area(const Surface_mesh& mesh, Surface_mesh::Face f);

/// compute the cotangent weight for edge e
double cotan_weight(const Surface_mesh& mesh, Surface_mesh::Edge e);

/// compute (mixed) Voronoi area of vertex v
double voronoi_area(const Surface_mesh& mesh, Surface_mesh::Vertex v);

/// compute barycentric Voronoi area of vertex v
double voronoi_area_barycentric(const Surface_mesh& mesh, Surface_mesh::Vertex v);

/// compute Laplace vector for vertex v (normalized by Voronoi area)
Point laplace(const Surface_mesh& mesh, Surface_mesh::Vertex v);

/// compute the sum of angles around vertex v (used for Gaussian curvature)
Scalar angle_sum(const Surface_mesh& mesh, Surface_mesh::Vertex v);

/// discrete curvature information for a vertex. used for vertex_curvature()
struct Vertex_curvature
{
    Vertex_curvature() : mean(0.0), gauss(0.0), max(0.0), min(0.0) {}

    Scalar mean;
    Scalar gauss;
    Scalar max;
    Scalar min;
};

/** compute min, max, mean, and Gaussian curvature for vertex v. this will not
 give realiable values for boundary vertices. */
Vertex_curvature vertex_curvature(const Surface_mesh& mesh, Surface_mesh::Vertex v);


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
