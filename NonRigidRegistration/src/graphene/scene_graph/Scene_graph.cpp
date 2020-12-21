//=============================================================================

#include <graphene/scene_graph/Scene_graph.h>
#include <graphene/scene_graph/Operations.h>
#include <graphene/scene_graph/Object_node.h>

#include <algorithm>

//=============================================================================

namespace graphene {
namespace scene_graph {

//=============================================================================


Scene_graph::
Scene_graph(gl::GL_state *glstate) :
    gl_state_(glstate)
{
    root_ = new Base_node("Root Node");
}


//-----------------------------------------------------------------------------


Scene_graph::
~Scene_graph()
{
    delete root_;
}

//-----------------------------------------------------------------------------

void
Scene_graph::
add_node(Base_node *node)
{
    node->init(gl_state_);
    root_->append(node);
}

//-----------------------------------------------------------------------------


void
Scene_graph::
clear()
{
    delete root_;
    root_ = new Base_node("Root Node");
}



//-----------------------------------------------------------------------------


void
Scene_graph::
traverse(Base_node* _node, Operation& _op) const
{
    _node->enter(gl_state_);

    _op.apply(_node, gl_state_);

    for (Base_node* child : _node->children())
    {
        traverse(child, _op);
    }

    _node->leave(gl_state_);
}


//-----------------------------------------------------------------------------


void
Scene_graph::
draw()
{
    Draw_operation op;
    traverse(root_, op);
}


//-----------------------------------------------------------------------------


void
Scene_graph::
draw_shadowmap(gl::GL_state *gl)
{
    Draw_shadowmap_operation op;

    //backup
    const Mat4f mvp_backup  = gl->modelviewproj_;
    const Mat4f mv_backup   = gl->modelview_;
    const Mat4f p_backup    = gl->proj_;
    const float near_backup = gl->near_;
    const float far_backup  = gl->far_;
    const float fovy_backup = gl->fovy_;


//    glEnable(GL_CULL_FACE); // only for closed meshes
//    glCullFace(GL_FRONT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.5f,10.0f);


    glDisable(GL_MULTISAMPLE);
    glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);

    gl::Shadow_map* sm;
    for (unsigned int i=0; i < gl->get_shadow_maps().size(); ++i)
    {
        sm = gl->get_shadow_maps()[i];

        gl->modelviewproj_ = sm->viewproj_ * gl->model_;
        gl->modelview_     = sm->view_ * gl->model_;
        gl->proj_          = sm->proj_;
        gl->near_          = sm->near_;
        gl->far_           = sm->far_;

        sm->activate();

        traverse(root_, op);

        sm->deactivate();
    }

    if (gl->multisampling_enabled_)
        glEnable(GL_MULTISAMPLE);

    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

    glDisable(GL_POLYGON_OFFSET_FILL);
//    glCullFace(GL_BACK);
//    glDisable(GL_CULL_FACE);

    //restore
    gl->modelviewproj_ = mvp_backup;
    gl->modelview_     = mv_backup;
    gl->proj_          = p_backup;
    gl->near_          = near_backup;
    gl->far_           = far_backup;
    gl->fovy_          = fovy_backup;
}

//-----------------------------------------------------------------------------


const Scene_graph::Bounding_box Scene_graph::bbox(bool recompute)
{
    if (recompute)
    {
        Compute_bounding_box_operation op;
        traverse(root_, op);
        bbox_ = op.bbox_;
        return op.bbox_;
    }
    else
    {
        return bbox_;
    }
}


//-----------------------------------------------------------------------------

Object_node *Scene_graph::select_node(const Vec3f &point) const
{
    Select_target_operation op(point);
    traverse(root_, op);
    return op.node_;
}

//-----------------------------------------------------------------------------

Object_node*
Scene_graph::
select_node(const std::string& name) const
{
    Select_target_by_name_operation op(name);
    traverse(root_, op);
    return op.node_;
}


//-----------------------------------------------------------------------------


Object_node*
Scene_graph::
selected_node() const
{
    Find_target_operation op;
    traverse(root_, op);
    return op.target_;
}


//-----------------------------------------------------------------------------


std::vector<Object_node*>
Scene_graph::
objects() const
{
    Collect_objects_operation op;
    traverse(root_, op);
    return op.object_nodes_;
}


//-----------------------------------------------------------------------------

Object_node*
Scene_graph::
make_exclusively_visible(const std::string &name) const
{
    std::vector<Object_node*> objs = objects();

    Object_node* visible_node = nullptr;

    for (size_t i=0; i < objs.size(); ++i)
    {
        Object_node* n = objs[i];
        if (n->name().find(name) != std::string::npos)
        {
            n->set_target(true);
            n->set_visible(true);
            visible_node = n;
        }
        else
        {
            n->set_target(false);
            n->set_visible(false);
        }
    }

    return visible_node;
}

//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
