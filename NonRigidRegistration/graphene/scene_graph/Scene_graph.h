//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_SCENE_GRAPH_H
#define GRAPHENE_SCENE_GRAPH_H
//=============================================================================


#include <graphene/geometry/Bounding_box.h>
#include <graphene/gl/GL_state.h>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


class Operation;
class Object_node;
class Base_node;


//=============================================================================


/// \addtogroup scene_graph scene_graph
/// @{

/// A simple scene graph for data managment and rendering
class Scene_graph
{

public:

    typedef graphene::geometry::Bounding_box Bounding_box;


public:

    Scene_graph(gl::GL_state* glstate);
    ~Scene_graph();

    void                draw();
    void                draw_shadowmap(gl::GL_state* gls);
    const Bounding_box bbox(bool recompute = true);

    ///select node which is closest to point
    Object_node* select_node(const Vec3f& point) const;
    ///select node which contains name
    Object_node* select_node(const std::string& name) const;
    ///return the currently selected node
    Object_node* selected_node() const;
    ///return a vector with all Object_nodes in the scene graph
    std::vector<Object_node*> objects() const;
    ///make the node which contains the string name the only visible node
    ///(also makes the node the current target)
    Object_node* make_exclusively_visible(const std::string& name) const;

    void add_node(Base_node* node);

    void clear();


private:

    void traverse(Base_node* _node, Operation& _op) const;


public:

    Base_node* root_;

private:
    gl::GL_state* gl_state_;

    Bounding_box bbox_;
};


//=============================================================================
/// @}
//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SCENE_GRAPH_H
//=============================================================================
