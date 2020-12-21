//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/types.h>
#include <graphene/gl/gl_includes.h>
#include <graphene/geometry/Geometry_representation.h>
#include <graphene/geometry/point_in_polygon.h>
#include <graphene/scene_graph/Object_node.h>

#include <algorithm>


//=============================================================================


namespace graphene {
namespace scene_graph {

using graphene::geometry::point_in_polygon;


//=============================================================================


Object_node::
Object_node(const std::string& name)
    : Base_node(name)
{
    draw_menu_.push_back("None");
    draw_mode_ = 0;

    draw_more_settings_menu_all_.push_back("None");
    draw_more_settings_active_.push_back(0);

    selection_menu_.push_back("None");
    selection_mode_ = 0;

    manipulation_menu_.push_back("None");
    manipulation_mode_ = 0;

    object_ = NULL;
}


//-----------------------------------------------------------------------------


Object_node::
~Object_node()
{
}


//-----------------------------------------------------------------------------


const std::vector<std::string>&
Object_node::
get_draw_modes_menu() const
{
    return draw_menu_;
}


//-----------------------------------------------------------------------------


void
Object_node::
clear_draw_modes()
{
    draw_menu_.clear();
    draw_mode_ = -1;
}


//-----------------------------------------------------------------------------


void
Object_node::
add_draw_mode(const std::string& s)
{
    // CHECK: Why use at(), when possible exception is not been caught?
    if (draw_menu_.size() == 1 && draw_menu_.at(0) == "None")
    {
        draw_menu_.erase(draw_menu_.begin());
    }

    if ( std::find(draw_menu_.begin(), draw_menu_.end(), s) == draw_menu_.end() )
    {
        draw_menu_.push_back(s);
    }
}


//-----------------------------------------------------------------------------


void
Object_node::
remove_draw_mode(const std::string& s)
{
    std::vector<std::string>::iterator it;
    it = std::find(draw_menu_.begin(),
                   draw_menu_.end(),
                   s);
    draw_menu_.erase(it);
}


//-----------------------------------------------------------------------------


void
Object_node::
set_draw_mode(int i)
{
    if (i < (int)draw_menu_.size())
        draw_mode_ = i;
    else
        std::cerr << "Failed to set draw mode to:" << i << std::endl;
}


//-----------------------------------------------------------------------------


void
Object_node::
set_draw_mode(const std::string& s)
{
    for (unsigned int i(0); i < draw_menu_.size(); i++)
    {
        if (draw_menu_[i] == s)
        {
            draw_mode_ = i;
            return;
        }
    }

    std::cerr << "Failed to set draw mode to:" << s << std::endl;
}


//-----------------------------------------------------------------------------


std::string&
Object_node::
get_draw_mode()
{
    return draw_menu_.at(draw_mode_);
}


//-----------------------------------------------------------------------------

int
Object_node::
get_draw_mode_idx()
{
    return draw_mode_;
}

//-----------------------------------------------------------------------------


const std::vector<std::string>&
Object_node::
get_draw_more_settings_menu_all() const
{
    return draw_more_settings_menu_all_;
}


//-----------------------------------------------------------------------------


void
Object_node::
clear_draw_more_settings()
{
    draw_more_settings_menu_all_.clear();
    draw_more_settings_active_.clear();

    draw_more_settings_menu_all_.push_back("None");
    draw_more_settings_active_.push_back(0);
}


//-----------------------------------------------------------------------------


void
Object_node::
add_draw_more_settings(const std::string& s)
{
    // CHECK: Why use at(), when possible exception is not been caught?
    if (draw_more_settings_menu_all_.size() == 1 && draw_more_settings_menu_all_.at(0) == "None")
    {
        draw_more_settings_menu_all_.erase(draw_more_settings_menu_all_.begin());

// TODO        if (draw_more_settings_active_.size() == 0)
//        {
            draw_more_settings_active_.clear();
//        }
    }

    if ( std::find(draw_more_settings_menu_all_.begin(), draw_more_settings_menu_all_.end(), s) == draw_more_settings_menu_all_.end() )
    {
        draw_more_settings_menu_all_.push_back(s);
    }
}


//-----------------------------------------------------------------------------


void
Object_node::
remove_draw_more_settings(const std::string& s)
{
    for (unsigned int i=0; i < draw_more_settings_menu_all_.size(); ++i)
    {
        if (draw_more_settings_menu_all_[i] == s)
        {
            for (unsigned int j = 0; j < draw_more_settings_active_.size(); ++j)
            {
                if (i == draw_more_settings_active_[j])
                {
                    draw_more_settings_active_.erase(draw_more_settings_active_.begin() + j);
                    break;
                }
            }

            draw_more_settings_menu_all_.erase(draw_more_settings_menu_all_.begin() + i);
            break;
        }
    }

    if (draw_more_settings_menu_all_.size() == 0)
    {
        draw_more_settings_menu_all_.push_back("None");
        draw_more_settings_active_.clear();
        draw_more_settings_active_.push_back(0);
    }
}


//-----------------------------------------------------------------------------


void
Object_node::
toggle_draw_more_settings(int i)
{
    std::vector<int>::iterator it;
    it = std::find(draw_more_settings_active_.begin(),
                   draw_more_settings_active_.end(),
                   i);

    if (it != draw_more_settings_active_.end())
    {
        draw_more_settings_active_.erase(it);
    }
    else
    {
        draw_more_settings_active_.push_back(i);
    }
}


//-----------------------------------------------------------------------------


std::vector<std::string>
Object_node::
get_draw_more_settings_active()
{
    std::vector<std::string> result;
    for (unsigned int i(0); i < draw_more_settings_active_.size(); i++)
    {
        result.push_back(draw_more_settings_menu_all_[draw_more_settings_active_[i]]);
    }

    return result;
}


//-----------------------------------------------------------------------------


const std::vector<std::string>&
Object_node::
get_selection_modes_menu() const
{
    return selection_menu_;
}


//-----------------------------------------------------------------------------


void
Object_node::
clear_selection_modes()
{
    selection_menu_.clear();
    selection_mode_ = -1;
}


//-----------------------------------------------------------------------------


void
Object_node::
add_selection_mode(const std::string& s)
{
    if (selection_menu_.size() == 1 && selection_menu_.at(0) == "None")
    {
        selection_menu_.erase(selection_menu_.begin());
    }

    selection_menu_.push_back(s);
}


//-----------------------------------------------------------------------------


void
Object_node::
remove_selection_mode(const std::string& s)
{
    std::vector<std::string>::iterator it;
    it = std::find(selection_menu_.begin(),
                   selection_menu_.end(),
                   s);
    selection_menu_.erase(it);
}


//-----------------------------------------------------------------------------


void
Object_node::
set_selection_mode(int i)
{
    selection_mode_ = i;
}


//-----------------------------------------------------------------------------


void
Object_node::
set_selection_mode(const std::string& s)
{
    for (unsigned int i(0); i < selection_menu_.size(); i++)
    {
        if (selection_menu_[i] == s)
        {
            selection_mode_ = i;
            return;
        }
    }

    std::cerr << "Failed to set selection mode to:" << s << std::endl;
}


//-----------------------------------------------------------------------------


std::string&
Object_node::
get_selection_mode()
{
    return selection_menu_.at(selection_mode_);
}


//-----------------------------------------------------------------------------


const std::vector<std::string>&
Object_node::
get_manipulation_modes_menu() const
{
    return manipulation_menu_;
}


//-----------------------------------------------------------------------------


void
Object_node::
clear_manipulation_modes()
{
    manipulation_menu_.clear();
    manipulation_mode_ = -1;
}


//-----------------------------------------------------------------------------


void
Object_node::
add_manipulation_mode(const std::string& s)
{
    if (manipulation_menu_.size() == 1 && manipulation_menu_.at(0) == "None")
    {
        manipulation_menu_.erase(manipulation_menu_.begin());
    }

    manipulation_menu_.push_back(s);
}


//-----------------------------------------------------------------------------


void
Object_node::
remove_manipulation_mode(const std::string& s)
{
    std::vector<std::string>::iterator it;
    it = std::find(manipulation_menu_.begin(),
                   manipulation_menu_.end(),
                   s);
    manipulation_menu_.erase(it);
}


//-----------------------------------------------------------------------------


void
Object_node::
set_manipulation_mode(int i)
{
    manipulation_mode_ = i;
}


//-----------------------------------------------------------------------------


void
Object_node::
set_manipulation_mode(const std::string& s)
{
    for (unsigned int i(0); i < manipulation_menu_.size(); i++)
    {
        if (manipulation_menu_[i] == s)
        {
            manipulation_mode_ = i;
            return;
        }
    }

    std::cerr << "Failed to set manipulation mode to:" << s << std::endl;
}


//-----------------------------------------------------------------------------


std::string&
Object_node::
get_manipulation_mode()
{
    return manipulation_menu_.at(manipulation_mode_);
}


//-----------------------------------------------------------------------------


float Object_node::select_target(const Vec3f &point)
{
    if (object_ != NULL)
    {
        float closest_dist = FLT_MAX;
        float d;
        for (size_t i=0; i < object_->points().size(); ++i)
        {
            d = distance(point,object_->points()[i]);
            if (d < closest_dist)
                closest_dist = d;
        }
        return closest_dist;
    }
    else
    {
        return FLT_MAX;
    }
}

//-----------------------------------------------------------------------------


std::vector<size_t>
Object_node::
select_lasso(std::vector<Vec2f> lasso, GL_state* gl_state, bool visible_only)
{
    const std::vector<Point> &points = this->object_->points();
    std::vector<size_t>       selected_points;
    Vec4f                     p(0);
    GLfloat                   z(0);
    GLint                     viewport[4];

    glGetIntegerv(GL_VIEWPORT, viewport);

    for (size_t i(0); i < points.size(); i++)
    {
        p = Vec4f(points[i][0], points[i][1], points[i][2], 1.0f);
        p = gl_state->modelviewproj_ * p;

        p = p / p[3];

        p[0] = (p[0] * 0.5f + 0.5f) * viewport[2];
        p[1] = (p[1] * 0.5f + 0.5f) * viewport[3];
        p[2] =  p[2] * 0.5f + 0.5f;

        if (visible_only)
        {
            glReadPixels(p[0], p[1], 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
        }
        else
        {
            z = FLT_MAX;
        }

        if (z > p[2])
        {
            if (point_in_polygon(Vec2f(p[0],p[1]),lasso))
            {
                selected_points.push_back(i);
            }
        }
    }

    return selected_points;
}


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
