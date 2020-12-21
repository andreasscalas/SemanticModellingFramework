//=============================================================================
// Copyright (C) 2013, 2014 by Graphics & Geometry Group, Bielefeld University
//=============================================================================


#include <graphene/gl/gl_includes.h>
#include <graphene/scene_graph/Lasso_selection_node.h>

#include <cfloat>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


Lasso_selection_node::
Lasso_selection_node(const std::string& _name)
    : Base_node(_name),
      vertexarrayobject_(0),
      vertexbuffer_(0)
{
}


//-----------------------------------------------------------------------------


Lasso_selection_node::
~Lasso_selection_node()
{
    if (vertexbuffer_)       glDeleteBuffers(1, &vertexbuffer_);
    if (vertexarrayobject_)  glDeleteVertexArrays(1, &vertexarrayobject_);
}


//-----------------------------------------------------------------------------


void
Lasso_selection_node::
draw(gl::GL_state *_gl)
{
    if (points_.size() > 0)
    {
        // setup orthogonal 2D projection
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);
        Mat4f matrix = Mat4f::ortho(0.0f,viewport[2], viewport[3], 0.0f, -0.1f, 1.0f);

        // setup shader
        gl::Shader* phong_shader = _gl->set_active_shader(graphene::gl::PHONG_SHADER);
        phong_shader->use();
        phong_shader->set_uniform("use_lighting", false);
        phong_shader->set_uniform("use_vertexcolor", false);
        phong_shader->set_uniform("use_texture1D",     false);
        phong_shader->set_uniform("use_texture2D",     false);
        phong_shader->set_uniform("front_color", Vec3f(1,0,0));
        phong_shader->set_uniform("modelview_projection_matrix", matrix);

        // draw lasso
        glBindVertexArray(vertexarrayobject_);
        glDrawArrays(GL_LINE_STRIP, 0, points_.size()+1);
        glBindVertexArray(0);

        //revert old state
        phong_shader->disable();
    }
}


//-----------------------------------------------------------------------------


geometry::Bounding_box
Lasso_selection_node::
bbox() const
{
    return Bounding_box();
}


//-----------------------------------------------------------------------------


void
Lasso_selection_node::
push_back(const Vec3f& p)
{
    points_.push_back(p);
    points_.push_back(points_[0]);

    upload_to_gl();

    if (!points_.empty())
    {
        points_.pop_back();
    }
}


//-----------------------------------------------------------------------------


void
Lasso_selection_node::
upload_to_gl()
{
    if (points_.empty()) return;

    if (!vertexarrayobject_) glGenVertexArrays(1, &vertexarrayobject_);
    glBindVertexArray(vertexarrayobject_);

    if (!vertexbuffer_) glGenBuffers(1,&vertexbuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer_);
    glBufferData(GL_ARRAY_BUFFER, points_.size()*3*sizeof(float), points_[0].data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);

    glBindVertexArray(0);
}


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================

