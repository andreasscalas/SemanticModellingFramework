#include <graphene/surface_mesh/algorithms/subdivision/sqrt3_subdivision.h>

namespace graphene {
namespace surface_mesh {

void sqrt3_subdivision(Surface_mesh& mesh)
{
    // reserve memory
    int nv = mesh.n_vertices();
    int ne = mesh.n_edges();
    int nf = mesh.n_faces();
    mesh.reserve(nv+nf, ne+3*nf, 3*nf);

    auto points = mesh.vertex_property<Point>("v:point");

    // iterators
    Surface_mesh::Vertex_iterator vit, vend=mesh.vertices_end();
    Surface_mesh::Face_iterator fit, fend=mesh.faces_end();
    Surface_mesh::Edge_iterator eit, eend=mesh.edges_end();


    // compute new positions of old vertices
    Surface_mesh::Vertex_property<Point> new_pos = mesh.add_vertex_property<Point>("v:np");
    for (vit=mesh.vertices_begin(); vit!=vend; ++vit)
    {
        if (!mesh.is_boundary(*vit))
        {
            Scalar n = mesh.valence(*vit);
            Scalar alpha = (4.0 - 2.0*cos(2.0*M_PI/n)) / 9.0;
            Point  p(0,0,0);
            Surface_mesh::Vertex_around_vertex_circulator vvit=mesh.vertices(*vit), vvend=vvit;

            do
            {
                p += points[*vvit];
            }
            while (++vvit != vvend);

            p = (1.0f-alpha)*points[*vit] + alpha/n*p;
            new_pos[*vit] = p;
        }
    }


    // split faces
    for (fit=mesh.faces_begin(); fit!=fend; ++fit)
    {
        Point  p(0,0,0);
        Scalar c(0);
        Surface_mesh::Vertex_around_face_circulator fvit = mesh.vertices(*fit), fvend=fvit;

        do
        {
            p += points[*fvit];
            ++c;
        }
        while (++fvit!=fvend);

        p /= c;

        mesh.split(*fit, p);
    }


    // set new positions of old vertices
    for (vit=mesh.vertices_begin(); vit!=vend; ++vit)
    {
        if (!mesh.is_boundary(*vit))
        {
            points[*vit] = new_pos[*vit];
        }
    }

    mesh.remove_vertex_property(new_pos);


    // flip old edges
    for (eit=mesh.edges_begin(); eit!=eend; ++eit)
    {
        if (mesh.is_flip_ok(*eit))
        {
            mesh.flip(*eit);
        }
    }
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

