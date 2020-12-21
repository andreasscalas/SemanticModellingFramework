//== INCLUDES =================================================================


#include <graphene/surface_mesh/algorithms/subdivision/linear_subdivision.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ==========================================================


void
linear_subdivision(Surface_mesh& mesh)
{    
    if (!mesh.is_triangle_mesh())
    {
        return;
    }


    // reserve memory
    int nv = mesh.n_vertices();
    int ne = mesh.n_edges();
    int nf = mesh.n_faces();
    mesh.reserve(nv+ne, 2*ne+3*nf, 4*nf);


    // get properties
    auto points = mesh.vertex_property<Point>("v:point");
    auto vpoint = mesh.add_vertex_property<Point>("loop:vpoint");
    auto epoint = mesh.add_edge_property<Point>("loop:epoint");


    // optional properties
    auto radius        = mesh.get_vertex_property<double>("v:radius");
    auto validity_mask = mesh.get_vertex_property<bool>("v:validity_mask");


    // compute edge positions
    for (auto e: mesh.edges())
    {
        epoint[e] = (points[mesh.vertex(e,0)] + points[mesh.vertex(e,1)]) * Scalar(0.5);
    }


    // insert new vertices on edges
    for (auto e: mesh.edges())
    {
        Surface_mesh::Vertex v0 = mesh.vertex(e,0);
        Surface_mesh::Vertex v1 = mesh.vertex(e,1);

        Surface_mesh::Halfedge h = mesh.insert_vertex(e, epoint[e]); // h points to new vertex on edge
        auto new_vertex = mesh.to_vertex(h);

        // validity mask
        if (validity_mask)
        {
            if ( validity_mask[v0] && validity_mask[v1] )
            {
                validity_mask[new_vertex] = true;
            }
            else
            {
                validity_mask[new_vertex] = false;
            }
        }

        // radius
        if (radius)
        {
            const double r0 = radius[v0];
            const double r1 = radius[v1];
            const double new_radius = (r0+r1) / 2.0;
            radius[new_vertex] = new_radius;
        }
    }


    // split faces
    Surface_mesh::Halfedge  h;
    for (auto f: mesh.faces())
    {
        h = mesh.halfedge(f);
        mesh.insert_edge(h, mesh.next_halfedge(mesh.next_halfedge(h)));
        h = mesh.next_halfedge(h);
        mesh.insert_edge(h, mesh.next_halfedge(mesh.next_halfedge(h)));
        h = mesh.next_halfedge(h);
        mesh.insert_edge(h, mesh.next_halfedge(mesh.next_halfedge(h)));
    }


    // clean-up properties
    mesh.remove_vertex_property(vpoint);
    mesh.remove_edge_property(epoint);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

