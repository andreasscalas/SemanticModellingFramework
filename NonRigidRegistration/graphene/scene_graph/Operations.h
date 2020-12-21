//=============================================================================
// Copyright (C) 2014 Graphics & Geometry Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_OPERATIONS_H
#define GRAPHENE_OPERATIONS_H
//=============================================================================


#include <graphene/scene_graph/Base_node.h>
#include <graphene/scene_graph/Object_node.h>
#include <graphene/gl/GL_state.h>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


class Operation
{
public:
    virtual ~Operation() {};
    virtual void apply(Base_node* _node, gl::GL_state* _gl) = 0;
};


//-----------------------------------------------------------------------------


class Draw_operation : public Operation
{
public:
    ~Draw_operation() {};

    void apply(Base_node* _node, gl::GL_state* _gl)
    {
        _node->draw(_gl);
    };
};


//-----------------------------------------------------------------------------

class Draw_shadowmap_operation : public Operation
{
public:
    ~Draw_shadowmap_operation() {}

    void apply(Base_node *_node, gl::GL_state *_gl)
    {
        _node->draw_shadowmap(_gl);
    }
};


//-----------------------------------------------------------------------------


class Compute_bounding_box_operation : public Operation
{
public:
    ~Compute_bounding_box_operation() {}

    void apply(Base_node* _node, gl::GL_state* _gl)
    {
        if (_node->visible())
            bbox_ += _node->bbox();
    }

    geometry::Bounding_box bbox_;
};


//-----------------------------------------------------------------------------


class Find_target_operation : public Operation
{
public:
    Find_target_operation() { target_ = NULL; };
    ~Find_target_operation() {};

    void apply(Base_node* node, gl::GL_state* _gl)
    {
        if (node->is_target())
        {
            Object_node* on = dynamic_cast<Object_node*>(node);

            if (on)
            {
                target_ = on;
            }
        }
    };

    Object_node* target_;
};

//-----------------------------------------------------------------------------

class Select_target_by_name_operation : public Operation
{
public:
    Select_target_by_name_operation(const std::string& name) :
        node_(nullptr),
        name_(name)
    {
    }
    ~Select_target_by_name_operation() {}

    void apply(Base_node* node, gl::GL_state* _gl)
    {
        if (node->name().find(name_) == 0)
        {
            Object_node* on = dynamic_cast<Object_node*>(node);

            if (on)
            {
                node_ = on;
                node_->set_target(true);
            }
        }
        else
        {
            node->set_target(false);
        }
    }

    Object_node* node_;
    const std::string& name_;
};


//-----------------------------------------------------------------------------


class Select_target_operation : public Operation
{
public:
    Select_target_operation(const Vec3f& point) :
        point_(point),
        closest_distance_(FLT_MAX),
        node_(NULL)
    { }
    ~Select_target_operation() {}

    void apply(Base_node* node, gl::GL_state* _gl)
    {
        Object_node* o_node = dynamic_cast<Object_node*>(node);
        if (o_node == NULL)
            return;

        o_node->set_target(false);

        const float dist = node->select_target(point_);
        if (dist < closest_distance_ && o_node->visible())
        {
            if (node_ != NULL)
                node_->set_target(false);

            node_ = o_node;
            closest_distance_ = dist;
            if (node_ != NULL)
                node_->set_target(true);
        }
    }

    Vec3f point_;
    float closest_distance_;
    Object_node* node_;
};


//-----------------------------------------------------------------------------


class Collect_objects_operation : public Operation
{
public:

    Collect_objects_operation() {}

    void apply(Base_node* node, gl::GL_state* _gl)
    {
        Object_node* on = dynamic_cast<Object_node*>(node);

        if (on)
        {
            object_nodes_.push_back(on);
        }
    };

    std::vector<Object_node*> object_nodes_;
};


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_OPERATIONS_H
//=============================================================================
