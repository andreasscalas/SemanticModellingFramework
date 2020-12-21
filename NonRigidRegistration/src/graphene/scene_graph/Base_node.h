//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_BASE_NODE_H
#define GRAPHENE_BASE_NODE_H
//=============================================================================


#include <graphene/geometry/Bounding_box.h>
#include <graphene/gl/GL_state.h>
#include <graphene/types.h>

#include <list>
#include <string>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================

/// \addtogroup scene_graph
/// @{

/// The base class for all scene graph nodes.
class Base_node
{

public:

    typedef geometry::Bounding_box Bounding_box;


public:

    Base_node(const std::string& name="<no name>");
    virtual ~Base_node();

    virtual void init(gl::GL_state* ){}

    virtual std::string& name();
    virtual Bounding_box bbox() const;
    virtual std::string info() const;
    virtual const std::string& fileinfo() const;
    virtual void set_fileinfo(const std::string& fileinfo);

    virtual float select_target(const Vec3f& point){return FLT_MAX;}

    virtual void draw(gl::GL_state* _gl) {}
    virtual void draw_shadowmap(gl::GL_state* gls) {}
    virtual void enter(gl::GL_state* _gl) {}
    virtual void leave(gl::GL_state* _gl) {}

    bool is_target() const { return is_target_; }
    void set_target(bool _b) { is_target_ = _b; }

    Base_node* parent() { return parent_; }
    const Base_node* parent() const { return parent_; }
    void set_parent(Base_node* parent) { parent_ = parent; }

    std::list<Base_node*>& children() { return children_; }
    const std::list<Base_node*>& children() const { return children_; }

    virtual void append(Base_node* node)
    {
        children_.push_back(node);
        node->set_parent(this);
    }

    bool visible() { return visible_; }
    void set_visible(bool v) { visible_ = v; }

protected:

    Base_node*            parent_;
    std::string           name_;
    std::string           fileinfo_;
    Bounding_box          bbox_;
    bool                  is_target_;
    bool                  visible_;
    std::list<Base_node*> children_;
};


//=============================================================================
/// @}
//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_BASE_NODE_H
//=============================================================================
