//== INCLUDES =================================================================

#include <graphene/surface_mesh/algorithms/subdivision/catmull_clark_subdivision.h>


//== NAMESPACE ================================================================

namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ==========================================================


void catmull_clark_subdivision(Surface_mesh& mesh)
{
    // reserve memory
    int nv = mesh.n_vertices();
    int ne = mesh.n_edges();
    int nf = mesh.n_faces();
    mesh.reserve(nv+ne+nf, 2*ne+4*nf, 4*nf);


    // get properties
    auto points   = mesh.vertex_property<Point>("v:point");
    auto vpoint   = mesh.add_vertex_property<Point>("catmull:vpoint");
    auto epoint   = mesh.add_edge_property<Point>("catmull:epoint");
    auto fpoint   = mesh.add_face_property<Point>("catmull:fpoint");
    auto vfeature = mesh.get_vertex_property<bool>("v:feature");
    auto efeature = mesh.get_edge_property<bool>("e:feature");


    // compute face vertices
    for (auto f: mesh.faces())
    {
        Point  p(0,0,0);
        Scalar c(0);
        for (auto v: mesh.vertices(f))
        {
            p += points[v];
            ++c;
        }
        p /= c;
        fpoint[f] = p;
    }


    // compute edge vertices
    for (auto e: mesh.edges())
    {
        // boundary or feature edge?
        if (mesh.is_boundary(e) || (efeature && efeature[e]))
        {
            epoint[e] = 0.5 * (points[mesh.vertex(e,0)] + points[mesh.vertex(e,1)]);
        }

        // interior edge
        else
        {
            Point p(0,0,0);                
            p += points[mesh.vertex(e,0)];
            p += points[mesh.vertex(e,1)];
            p += fpoint[mesh.face(e,0)];
            p += fpoint[mesh.face(e,1)];
            p *= 0.25;
            epoint[e] = p;
        }
    }


    // compute new positions for old vertices
    for (auto v: mesh.vertices())
    {
        // isolated vertex?
        if (mesh.is_isolated(v))
        {
            vpoint[v] = points[v];
        }

        // boundary vertex?
        else if (mesh.is_boundary(v))
        {
            Surface_mesh::Halfedge h1 = mesh.halfedge(v);
            Surface_mesh::Halfedge h0 = mesh.prev_halfedge(h1);

            Point p = points[v];
            p *= 6.0;
            p += points[mesh.to_vertex(h1)];
            p += points[mesh.from_vertex(h0)];
            p *= 0.125;

            vpoint[v] = p;
        }

        // interior feature vertex?
        else if (vfeature && vfeature[v])
        {
            Point p = points[v];
            p *= 6.0;
            int count(0);

            for (auto h: mesh.halfedges(v))
            {
                if (efeature[mesh.edge(h)])
                {
                    p += points[mesh.to_vertex(h)];
                    ++count;
                }
            }

            if (count == 2) // vertex is on feature edge
            {
                p *= 0.125;
                vpoint[v] = p;
            }
            else // keep fixed
            {
                vpoint[v] = points[v];
            }
        }

        // interior vertex
        else
        {
            // weights from SIGGRAPH paper "Subdivision Surfaces in Character Animation"

            const Scalar k = mesh.valence(v);
            Point p(0,0,0);

            for (auto vv: mesh.vertices(v))
                p += points[vv];

            for (auto f: mesh.faces(v))
                p += fpoint[f];

            p /= (k*k);

            p += ((k-2.0)/k) * points[v];

            vpoint[v] = p;
        }
    }


    // assign new positions to old vertices
    for (auto v: mesh.vertices())
    {
        points[v] = vpoint[v];
    }


    // split edges
    for (auto e: mesh.edges())
    {
        // feature edge?
        if (efeature && efeature[e])
        {
            Surface_mesh::Halfedge h  = mesh.insert_vertex(e, epoint[e]);
            Surface_mesh::Vertex   v  = mesh.to_vertex(h);
            Surface_mesh::Edge     e0 = mesh.edge(h);
            Surface_mesh::Edge     e1 = mesh.edge(mesh.next_halfedge(h));
            vfeature[v]  = true;
            efeature[e0] = true;
            efeature[e1] = true;
        }

        // normal edge
        else
        {
            mesh.insert_vertex(e, epoint[e]);
        }
    }


    // split faces
    for (auto f: mesh.faces())
    {
        Surface_mesh::Halfedge h0 = mesh.halfedge(f);
        mesh.insert_edge(h0, mesh.next_halfedge(mesh.next_halfedge(h0)));
        Surface_mesh::Halfedge h1 = mesh.next_halfedge(h0);
        mesh.insert_vertex(mesh.edge(h1), fpoint[f]);

        Surface_mesh::Halfedge h = mesh.next_halfedge(mesh.next_halfedge(mesh.next_halfedge(h1)));
        while (h != h0)
        {
            mesh.insert_edge(h1, h);
            h = mesh.next_halfedge(mesh.next_halfedge(mesh.next_halfedge(h1)));
        }
    }


    // clean-up properties
    mesh.remove_vertex_property(vpoint);
    mesh.remove_edge_property(epoint);
    mesh.remove_face_property(fpoint);
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

