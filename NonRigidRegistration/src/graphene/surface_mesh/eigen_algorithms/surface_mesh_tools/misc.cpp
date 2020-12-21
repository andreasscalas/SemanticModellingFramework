//=============================================================================
// Copyright (C) 2014 Graphics & Geometry Group, Bielefeld University
//=============================================================================


#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>


//=============================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


Scalar mean_edge_length(const Surface_mesh& mesh)
{
    Scalar mean = 0;

    for (auto e : mesh.edges())
    {
        mean += mesh.edge_length(e);
    }

    mean /= (Scalar)mesh.n_edges();

    return mean;
}


//-----------------------------------------------------------------------------


Point centroid(Surface_mesh& mesh, const Surface_mesh::Face& f)
{
    auto points = mesh.vertex_property<Point>("v:point");

    Point centroid(0);
    size_t n_vertices(0);

    for (auto v : mesh.vertices(f))
    {
        centroid += points[v];
        n_vertices++;
    }

    centroid /= (Scalar)n_vertices;

    return centroid;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
