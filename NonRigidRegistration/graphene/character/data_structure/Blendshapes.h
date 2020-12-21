//=============================================================================

#ifndef GRAPHENE_BLENDSHAPES_H
#define GRAPHENE_BLENDSHAPES_H

//=============================================================================

#include <graphene/character/data_structure/Surface_mesh_skin.h>

//=============================================================================

namespace graphene {
namespace character {
//=============================================================================


class Blendshapes
{

public:
    class Target_shape : public surface_mesh::Surface_mesh
    {
    public:
        Surface_mesh::Vertex_property<Point>    vertices_;
        Surface_mesh::Vertex_property<Normal>   v_normals_;
        Surface_mesh::Halfedge_property<Normal> h_normals_;

        void init_properties()
        {
            vertices_  = get_vertex_property<Point>("v:point");
            v_normals_ = get_vertex_property<Normal>("v:normal");
            h_normals_ = get_halfedge_property<Normal>("h:normal");
        }
    };

private:
    Surface_mesh_skin* base_;
    surface_mesh::Surface_mesh::Vertex_property<Point> blended_vertices_;
    surface_mesh::Surface_mesh::Vertex_property<Normal> blended_v_normals_;
    surface_mesh::Surface_mesh::Halfedge_property<Normal> blended_h_normals_;

    std::vector<Target_shape*> targets_;

    std::vector<float> active_weights_;
    std::vector<unsigned int> active_indices_;

    std::vector<unsigned int> moving_indices_;

public:
    Blendshapes();
    ~Blendshapes();

    void clear();

    void clear_blended();

    void set_base(Surface_mesh_skin* base) {base_ = base;}
    Surface_mesh_skin* base() { return base_; }
    const Surface_mesh_skin* base() const  { return base_; }


    const std::vector<Target_shape*>& targets() const { return targets_; }
    std::vector<Target_shape*>& targets() { return targets_; }
    std::vector<float>&        active_weights() { return active_weights_; }
    std::vector<unsigned int>& active_indices() { return active_indices_; }

    void add_target(Target_shape* target);

    void set_active(const std::vector<float>& weights, const std::vector<unsigned int> &indices);
    void clear_active();
    bool apply();

    void update_moving_indizes();

};


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

