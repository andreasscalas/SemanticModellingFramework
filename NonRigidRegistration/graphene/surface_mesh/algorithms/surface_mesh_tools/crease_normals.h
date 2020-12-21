//=============================================================================
#ifndef GRAPHENE_CREASE_NORMALS_H
#define GRAPHENE_CREASE_NORMALS_H
//=============================================================================

//== INCLUDES ===================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


//=============================================================================
namespace graphene
{
namespace surface_mesh
{
static void crease_normals(Surface_mesh* mesh, float crease_angle = 0.0f, const std::string& property_name = "")
{
    bool is_triangle = mesh->is_triangle_mesh();


    if (!property_name.empty() && crease_angle >= 0.001f && is_triangle)
    {
        const Scalar crease = cos( crease_angle / 180.0 * M_PI );
        Normal n, ni, nni;
        Surface_mesh::Vertex v1, v2;
        Surface_mesh::Halfedge_around_face_circulator hfc;
        Point e1, e2;
        Scalar w;
        auto points = mesh->vertex_property<Point>("v:point");

        Surface_mesh::Halfedge_property<Normal> hnormals = mesh->halfedge_property<Normal>(property_name);

        mesh->update_face_normals();
        auto fnormals = mesh->face_property<Normal>("f:normal");

        for (auto f : mesh->faces())
        {
            ni = fnormals[f];

            hfc = mesh->halfedges(f);

            for (auto v : mesh->vertices(f))
            {
                n = 0.0;

                for (auto h : mesh->halfedges(v))
                {
                    if (!mesh->is_boundary(h))
                    {
                        nni = fnormals[ mesh->face(h) ];

                        if (dot(ni,nni) > crease)
                        {
                            v1  = mesh->to_vertex(h);
                            v2  = mesh->to_vertex(mesh->prev_halfedge(mesh->prev_halfedge(h)));
                            e1  = (points[v1] - points[v]).normalize();
                            e2  = (points[v2] - points[v]).normalize();
                            w   = acos( std::max(-1.0f, std::min(1.0f, dot(e1, e2) )));
                            n  += w * nni;
                        }
                    }
                }

                hnormals[*hfc] = n.normalize();
                ++hfc;

                //vertex_normals.push_back(n.normalize());
            }
        }
    }
    else
    {
        Surface_mesh::Halfedge_property<Normal> hnormals = mesh->get_halfedge_property<Normal>(property_name);
        if (hnormals)
        {
            mesh->remove_halfedge_property<Normal>(hnormals);
        }

        if (is_triangle)
            mesh->update_vertex_normals();
        else
            mesh->update_face_normals();
    }
}

}
}



//=============================================================================
#endif // GRAPHENE_MEANVALUE_COORDS_H
//=============================================================================

