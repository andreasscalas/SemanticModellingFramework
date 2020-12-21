//=============================================================================

#ifndef GRAPHENE_SURFACE_MESH_SKIN_H
#define GRAPHENE_SURFACE_MESH_SKIN_H

//=============================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Bounding_box.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


class Surface_mesh_skin : public surface_mesh::Surface_mesh
{
public:
    Surface_mesh::Vertex_property<Point>                vertices_;
    Surface_mesh::Vertex_property<Color>                v_colors_;
    Surface_mesh::Vertex_property<Normal>               v_normals_;
    Surface_mesh::Vertex_property<Vec4f>                v_depends1_;
    Surface_mesh::Vertex_property<Vec4f>                v_weights1_;
    Surface_mesh::Vertex_property<Vec4f>                v_depends2_;
    Surface_mesh::Vertex_property<Vec4f>                v_weights2_;

    Surface_mesh::Halfedge_property<Texture_coordinate> h_texcoords_;
    Surface_mesh::Halfedge_property<Normal>             h_normals_;

    bool visible_;
    float alpha_;
    Vec4f material_;

public:

    Surface_mesh_skin();
    Surface_mesh_skin(const Surface_mesh& mesh);
    virtual ~Surface_mesh_skin();

    void init_properties();

    const geometry::Bounding_box bbox();

};


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

