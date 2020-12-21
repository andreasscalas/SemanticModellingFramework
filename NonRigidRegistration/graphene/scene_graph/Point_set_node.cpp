//=============================================================================
// Copyright (C) 2014, 2015 Graphics & Geometry Group, Bielefeld University
//=============================================================================


#include <graphene/scene_graph/Point_set_node.h>
#include <graphene/gl/gl_includes.h>
#include <graphene/gl/Shader.h>
#include <graphene/macros.h>

#include <algorithm>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


Point_set_node::
Point_set_node(const std::string& _name)
    : Object_node(_name),
      radius_(4.0f),
      color_(0.0f),
      selection_color_(1.0f,0.1f,0.1f)
{
    this->object_ = &point_set_;

    // init draw modes
    clear_draw_modes();
    add_draw_mode("Points");
    add_draw_mode("Spheres");
    set_draw_mode(0);

    clear_selection_modes();
    add_selection_mode("Lasso");
    add_selection_mode("Constraints");
    add_selection_mode("Landmarks");
    set_selection_mode(0);

    selection_highlight_idx_ = -1;

    // initialize GL buffers to zero
    vertex_array_object_ = 0;
    vertex_buffer_       = 0;
    normal_buffer_       = 0;
    color_buffer_        = 0;

    //selection
    selection_index_buffer_ = 0;

    sphere_radius_ = 1.0f;

    front_color_ = Color(0.4f, 0.425f, 0.475f);
    material_    = Vec4f(0.1f, 0.8f, 0.0f, 16.0f);
}


//-----------------------------------------------------------------------------


Point_set_node::
~Point_set_node()
{
    // delete all OpenGL buffers
    if (vertex_buffer_)           glDeleteBuffers(1, &vertex_buffer_);
    if (normal_buffer_)           glDeleteBuffers(1, &normal_buffer_);
    if (color_buffer_ )           glDeleteBuffers(1, &color_buffer_);
    if (selection_index_buffer_)  glDeleteBuffers(1, &selection_index_buffer_);
    if (vertex_array_object_) glDeleteVertexArrays(1, &vertex_array_object_);

}


//-----------------------------------------------------------------------------


bool
Point_set_node::
load(const std::string& filename)
{
    fileinfo_ = filename;

    // extract file extension
    std::string::size_type dot(filename.rfind("."));
    if (dot == std::string::npos) return false;
    std::string ext = filename.substr(dot+1, filename.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), tolower);

    bool ok = false;

    // extension determines reader
    if (ext == "xyz")
    {
        ok = point_set_.read(filename.c_str());
    }
    else if (ext == "cxyz")
    {
        ok = point_set_.read_cxyz(filename.c_str());
    }
    else if (ext == "txt")
    {
        ok = point_set_.read_photoscan_txt(filename.c_str());
    }

    update_point_set();

    return ok;
}


//-----------------------------------------------------------------------------


bool
Point_set_node::
save(const std::string& filename) const
{
    // extract file extension
    std::string::size_type dot(filename.rfind("."));
    if (dot == std::string::npos) return false;
    std::string ext = filename.substr(dot+1, filename.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), tolower);

    bool ok = false;

    // extension determines reader
    if (ext == "xyz")
    {
        ok = point_set_.write(filename.c_str());
    }
    else if(ext == "cxyz")
    {
        ok = point_set_.write_cxyz(filename.c_str());
    }
    else if(ext == "txt")
    {
        ok = point_set_.write_photoscan_txt(filename.c_str());
    }

    LOG(Log_info) << filename  << " saved!\n";

    return ok;
}


//-----------------------------------------------------------------------------


std::string
Point_set_node::
info() const
{
    std::ostringstream s;
    s << point_set_.size() << " points";
    return s.str();
}

void
Point_set_node::
set_radius(const float &radius)
{
    radius_ = radius;
    //sphere_radius_ = radius * 1000.0f * (bbox_.size() / point_set_.points_.size());

    const size_t step = point_set_.points_.size() / 10;
    std::set<float> nn;
    float min_d;

    for (size_t i = 0; i < point_set_.points_.size() && i < 100000000; i += step)
    {
        min_d = FLT_MAX;
        for (size_t j=0; j < point_set_.points_.size(); ++j)
        {
            if (j != i)
            {
                const float d = distance(point_set_.points_[j], point_set_.points_[i]);
                if (d < min_d)
                {
                    min_d = d;
                }
            }
        }

        nn.insert(min_d);
    }

    std::set<float>::iterator sit;
    float avg = 0.0f;
    int c=0;
    for (sit = nn.begin(), c=0; sit != nn.end() && c < 5; ++sit, ++c)
    {
        avg += *sit;
    }
    avg /= c;

    sphere_radius_ = radius * avg;
}


//-----------------------------------------------------------------------------


void
Point_set_node::
draw(gl::GL_state* gl_state)
{
    if (point_set_.points_.empty() || !visible_)
    {
        return;
    }


    Vec3f front_color = front_color_;
    if (is_target_)
        front_color *= Vec3f(0.8f,1.0f,0.8f);


    if (draw_menu_[draw_mode_] == "Points")
    {
        Vec4f viewport;
        glGetFloatv(GL_VIEWPORT, viewport.data());
        const float t = gl_state->near_ * tan( gl_state->fovy_ * M_PI / 360.0 );
        const Vec4f nhtb(gl_state->near_, viewport[3], t, -t);


        // setup Phong shader
        gl::Shader* point_shader = gl_state->set_active_shader(gl::POINT_SHADER);
        point_shader->set_uniform("front_color",     front_color);
        point_shader->set_uniform("modelview_projection_matrix", gl_state->modelviewproj_);
        point_shader->set_uniform("modelview_matrix", gl_state->modelview_);
        point_shader->set_uniform("normal_matrix", gl_state->normal_);
        point_shader->set_uniform("material", material_);

        point_shader->set_uniform("use_lighting",    ! point_set_.normals_.empty());
        point_shader->set_uniform("use_vertexcolor", ! point_set_.colors_.empty());
        point_shader->set_uniform("nhtb", nhtb);
        point_shader->set_uniform("radius", sphere_radius_);

        glBindVertexArray(vertex_array_object_);


        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

        glDepthRange(0.002, 1.0);
        //glPointSize(radius_);
        glDrawArrays(GL_POINTS, 0, (GLsizei)point_set_.size());

        draw_selection_points(gl_state);

        glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    }

    if (draw_menu_[draw_mode_] == "Spheres")
    {
        // collect uniforms for sphere and cylinder shader
        Vec4f viewport;
        glGetFloatv(GL_VIEWPORT, viewport.data());
        const float wv = viewport[2];
        const float hv = viewport[3];
        const float n  = gl_state->near_;
        const float f  = gl_state->far_;
        const Vec2f width_height(wv, hv);
        const Vec2f near_far(n, f/(f-n));
        const Mat4f tMVP = transpose(gl_state->modelviewproj_);
        const Mat4f iMV  = inverse(gl_state->modelview_);
        const Mat4f vp   = Mat4f::viewport(viewport[0],
                                           viewport[1],
                                           viewport[2],
                                           viewport[3]) * gl_state->proj_;
        const Mat4f iVP  = inverse(vp);

        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

        // spheres
        gl::Shader* sphere_shader = gl_state->set_active_shader(gl::SPHERE_SHADER);
        sphere_shader->set_uniform("material",      front_color);
        sphere_shader->set_uniform("mvp_transpose", tMVP);
        sphere_shader->set_uniform("mv_inverse",    iMV);
        sphere_shader->set_uniform("width_height",  width_height);
        sphere_shader->set_uniform("near_far",      near_far);
        sphere_shader->set_uniform("VP_inverse",    iVP);

        glBindVertexArray(vertex_array_object_);

        if (point_set_.colors_.empty())
            glVertexAttrib3f(gl::attrib_locations::COLOR, 0.5f,0.5f,0.5f);

        glVertexAttrib1f(gl::attrib_locations::RADIUS, sphere_radius_);
        glDepthRange(0.001, 1.0);
        glDrawArrays(GL_POINTS, 0, (GLsizei)point_set_.size());

        draw_selection_spheres(gl_state);
    }


    // reset stuff
    glBindVertexArray(0);
    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glPointSize(1.0);
    glDepthRange(0.0, 1.0);
}


//-----------------------------------------------------------------------------


geometry::Bounding_box
Point_set_node::
bbox() const
{
    return bbox_;
}


//-----------------------------------------------------------------------------


void
Point_set_node::
initialize_buffers()
{
    // initialize buffers
    if (vertex_array_object_ == 0)
    {
        glGenVertexArrays(1, &vertex_array_object_);
        glBindVertexArray(vertex_array_object_);

        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &normal_buffer_);
        glGenBuffers(1, &color_buffer_);
        glGenBuffers(1, &selection_index_buffer_);

        glBindVertexArray(0);
    }

}


//-----------------------------------------------------------------------------



void
Point_set_node::
update_point_set()
{
    // initialize buffers
    if (vertex_array_object_ == 0)
    {
        initialize_buffers();
    }

    bbox_ = Bounding_box();

    for (Point p : point_set_.points_)
    {
        bbox_ += p;
    }

    set_radius(radius_);


    glBindVertexArray(vertex_array_object_);

    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, point_set_.size()*3*sizeof(float),
                 point_set_.points_.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);


    // normals
    if (! point_set_.normals_.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
        glBufferData(GL_ARRAY_BUFFER, point_set_.normals_.size()*3*sizeof(float),
                     point_set_.normals_.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(gl::attrib_locations::NORMAL, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(gl::attrib_locations::NORMAL);
    }

    if (! point_set_.colors_.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
        glBufferData(GL_ARRAY_BUFFER, point_set_.colors_.size()*3*sizeof(float),
                     point_set_.colors_.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(gl::attrib_locations::COLOR, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(gl::attrib_locations::COLOR);
    }
    else
    {
        glVertexAttrib3fv(gl::attrib_locations::COLOR, color_.data());
        glDisableVertexAttribArray(gl::attrib_locations::COLOR);
    }

    glVertexAttrib1f(gl::attrib_locations::RADIUS, sphere_radius_);

    glBindVertexArray(0);

    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
clear_selections()
{
    point_set_.selected_indices_.clear();
    update_selection();
}


//-----------------------------------------------------------------------------

void
Point_set_node::
invert_selections()
{
    if (point_set_.points_.size() == point_set_.selected_indices_.size())
    {
        point_set_.selected_indices_.clear();
        update_selection();
        return;
    }
    else if (point_set_.selected_indices_.empty())
    {
        for (size_t i=0; i < point_set_.points_.size(); ++i)
        {
            point_set_.selected_indices_.push_back((unsigned int) i);
        }
        update_selection();
        return;
    }


    size_t i,inv_size;

    //get future size of selected indices
    inv_size = point_set_.points_.size() - point_set_.selected_indices_.size();

    //use temp vector to store selected points
    std::vector<bool> is_selected(point_set_.points_.size(), false);
    for (i=0; i < point_set_.selected_indices_.size(); ++i)
    {
        is_selected[point_set_.selected_indices_[i]] = true;
    }

    point_set_.selected_indices_.clear();
    point_set_.selected_indices_.reserve(inv_size);

    //iterate over marked points and push back all non-selected point indices
    for (i=0; i < is_selected.size(); ++i)
    {
        if (!is_selected[i])
        {
            point_set_.selected_indices_.push_back((unsigned int) i);
        }
    }

    update_selection();
}


//-----------------------------------------------------------------------------

void
Point_set_node::
select_all()
{
    point_set_.selected_indices_.clear();
    size_t i;
    for (i=0; i < point_set_.points_.size(); ++i)
    {
        point_set_.selected_indices_.push_back((unsigned int)i);
    }

    update_selection();
}


//-----------------------------------------------------------------------------


void
Point_set_node::
delete_selected()
{
    if (point_set_.selected_indices_.empty())
        return;

    size_t i;
    const std::vector<Point>  points(point_set_.points_);
    const std::vector<Normal> normals(point_set_.normals_);
    const std::vector<Color>  colors(point_set_.colors_);


    //use temp vector to store selected points
    std::vector<bool> is_selected(point_set_.points_.size(), false);
    for (i=0; i < point_set_.selected_indices_.size(); ++i)
    {
        is_selected[point_set_.selected_indices_[i]] = true;
    }

    std::vector<Point>  lm_points_;
    lm_points_.reserve(point_set_.lm_ears_.size());
    std::vector<Normal> lm_normals_;
    lm_normals_.reserve(point_set_.lm_ears_.size());
    std::vector<Color>  lm_colors_;
    lm_colors_.reserve(point_set_.lm_ears_.size());
    for (i=0; i < point_set_.landmarks_.size(); ++i)
    {
        is_selected[point_set_.landmarks_[i]] = true;

        lm_points_.push_back(points[point_set_.landmarks_[i]]);
        if (!normals.empty())
            lm_normals_.push_back(normals[point_set_.landmarks_[i]]);
        if (!colors.empty())
            lm_colors_.push_back(colors[point_set_.landmarks_[i]]);
    }
    point_set_.landmarks_.clear();

    std::vector<Point>  lm_ears_points_;
    lm_ears_points_.reserve(point_set_.lm_ears_.size());
    std::vector<Normal> lm_ears_normals_;
    lm_ears_normals_.reserve(point_set_.lm_ears_.size());
    std::vector<Color>  lm_ears_colors_;
    lm_ears_colors_.reserve(point_set_.lm_ears_.size());
    for (i=0; i < point_set_.lm_ears_.size(); ++i)
    {
        is_selected[point_set_.lm_ears_[i]] = true;

        lm_ears_points_.push_back(points[point_set_.lm_ears_[i]]);
        if (!normals.empty())
            lm_ears_normals_.push_back(normals[point_set_.lm_ears_[i]]);
        if (!colors.empty())
            lm_ears_colors_.push_back(colors[point_set_.lm_ears_[i]]);
    }
    point_set_.lm_ears_.clear();

    const size_t new_size = point_set_.points_.size() - point_set_.selected_indices_.size();

    point_set_.points_.clear();
    point_set_.normals_.clear();
    point_set_.colors_.clear();



    //reserve memory with new size
    point_set_.points_.reserve(new_size);
    if (!normals.empty()) //if there are no normals before, we won't need memory for them after
        point_set_.normals_.reserve(new_size);
    if (!colors.empty()) //if there are no colors before, we won't need memory for them after
        point_set_.colors_.reserve(new_size);

    for (i=0; i < is_selected.size(); ++i)
    {
        if (!is_selected[i])
        {
            point_set_.points_.push_back(points[i]);

            if (!normals.empty())
                point_set_.normals_.push_back(normals[i]);

            if (!colors.empty())
                point_set_.colors_.push_back(colors[i]);
        }
    }
    point_set_.selected_indices_.clear();


    size_t new_ps_size_wo_lm = point_set_.points_.size();
    for (size_t i = 0; i < lm_points_.size(); ++i)
    {
        point_set_.points_.push_back( lm_points_[i] );
        if (!lm_normals_.empty())
            point_set_.normals_.push_back( lm_normals_[i] );
        if (!lm_colors_.empty())
            point_set_.colors_.push_back( lm_colors_[i] );

        point_set_.landmarks_.push_back( new_ps_size_wo_lm + i );
    }

    new_ps_size_wo_lm = point_set_.points_.size();
    for (size_t i = 0; i < lm_ears_points_.size(); ++i)
    {
        point_set_.points_.push_back( lm_ears_points_[i] );
        if (!lm_ears_normals_.empty())
            point_set_.normals_.push_back( lm_ears_normals_[i] );
        if (!lm_ears_colors_.empty())
            point_set_.colors_.push_back( lm_ears_colors_[i] );

        point_set_.lm_ears_.push_back( new_ps_size_wo_lm + i );
    }


    update_point_set();
    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
grow_selection()
{
    std::cerr << "Point_set_node::grow_selection: [WARNING] Not Implemented!" << std::endl;
}

//-----------------------------------------------------------------------------

void
Point_set_node::
set_selection(const std::vector<size_t>& indices)
{
    point_set_.selected_indices_.clear();
    point_set_.selected_indices_.resize(indices.size());

    size_t i;
    for (i=0; i < indices.size(); ++i)
    {
        point_set_.selected_indices_[i] = (unsigned int)indices[i];
    }

    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
add_selection(const std::vector<size_t>& indices)
{
    size_t old_size = point_set_.selected_indices_.size();
    point_set_.selected_indices_.reserve(indices.size() + point_set_.selected_indices_.size());

    size_t i,j;
    for (i=0; i < indices.size(); ++i)
    {
        const unsigned int new_idx = (unsigned int)indices[i];
        for (j=0; j < old_size; ++j)
        {
            if (point_set_.selected_indices_[j] == new_idx)
            {
                break;
            }
        }
        if (j == old_size)
        {
            point_set_.selected_indices_.push_back(new_idx);
        }
    }

    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
get_selection(std::vector<size_t>& indices)
{
    indices.clear();
    indices.resize(point_set_.selected_indices_.size());

    size_t i;
    for (i=0; i < point_set_.selected_indices_.size(); ++i)
    {
        indices[i] = point_set_.selected_indices_[i];
    }

    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
clear_selection(const std::vector<size_t>& indices)
{

    size_t i;

    //use temp vector to store selected points
    std::vector<bool> is_selected(point_set_.points_.size(), false);
    for (i=0; i < point_set_.selected_indices_.size(); ++i)
    {
        is_selected[point_set_.selected_indices_[i]] = true;
    }

    //"unselect" indices
    for (i=0; i < indices.size(); ++i)
    {
        is_selected[indices[i]] = false;
    }

    //clear selected
    point_set_.selected_indices_.clear();
    //fill indices vector with currently selected
    for (i=0; i < is_selected.size(); ++i)
    {
        if (is_selected[i])
            point_set_.selected_indices_.push_back((unsigned int) i);
    }

    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
select_point(Point p)
{
    float d, d_min(FLT_MAX);

    size_t i;
    unsigned int i_min;

    for (i=0; i < point_set_.points_.size(); ++i)
    {
        d = distance(point_set_.points_[i], p);
        if (d < d_min)
        {
            i_min = (unsigned int) i;
            d_min = d;
        }
    }
    point_set_.selected_indices_.push_back(i_min);
    update_selection();

}

//-----------------------------------------------------------------------------

void
Point_set_node::
select_landmark(const Point &p)
{
    float d, d_min(FLT_MAX);

    size_t i;
    unsigned int i_min;

    for (i=0; i < point_set_.points_.size(); ++i)
    {
        d = distance(point_set_.points_[i], p);
        if (d < d_min)
        {
            i_min = (unsigned int) i;
            d_min = d;
        }
    }
    point_set_.selected_indices_.push_back(i_min);
    point_set_.landmarks_.push_back(i_min);
    update_selection();

}

//-----------------------------------------------------------------------------

void Point_set_node::undo_last_landmark()
{
    if (point_set_.landmarks_.empty())
        return;

    if (point_set_.landmarks_.size() == point_set_.selected_indices_.size())
    {
        point_set_.landmarks_.pop_back();
        point_set_.selected_indices_.pop_back();
        update_selection();
    }
    else
    {
        point_set_.landmarks_.pop_back();
    }

}

//-----------------------------------------------------------------------------

void
Point_set_node::
move_landmark(const int current_landmark_idx, const Point &p)
{
    float d, d_min(FLT_MAX);

    size_t i;
    unsigned int i_min;

    for (i=0; i < point_set_.points_.size(); ++i)
    {
        d = distance(point_set_.points_[i], p);
        if (d < d_min)
        {
            i_min = (unsigned int) i;
            d_min = d;
        }
    }
    point_set_.selected_indices_[current_landmark_idx] = i_min;
    point_set_.landmarks_[current_landmark_idx] = i_min;
    highlight_landmark(current_landmark_idx);
    update_selection();
}

//-----------------------------------------------------------------------------

void
Point_set_node::
clear_landmarks()
{
    point_set_.landmarks_.clear();
    selection_highlight_idx_ = -1;
}

//-----------------------------------------------------------------------------

void
Point_set_node::
highlight_landmark(int idx)
{
    if (point_set_.landmarks_.empty() || idx == -1)
    {
        selection_highlight_idx_ = -1;
        return;
    }
    selection_highlight_idx_ = (int)point_set_.landmarks_[idx];
}

//-----------------------------------------------------------------------------

int
Point_set_node::
get_num_landmarks()
{
    return (int) point_set_.landmarks_.size();
}


//-----------------------------------------------------------------------------

void
Point_set_node::
update_selection()
{
    if (point_set_.selected_indices_.empty())
        return;

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, selection_index_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, point_set_.selected_indices_.size()*sizeof(unsigned int), &point_set_.selected_indices_[0], GL_STATIC_DRAW);
    glBindVertexArray(0);
}

//-----------------------------------------------------------------------------

void
Point_set_node::
draw_selection_points(GL_state* gl_state)
{
    if (point_set_.selected_indices_.empty())
        return;

    gl::Shader* phong_shader = gl_state->get_active_shader();
    phong_shader->set_uniform("front_color",  selection_color_);
    phong_shader->set_uniform("use_lighting", false);
    phong_shader->set_uniform("use_vertexcolor", false);
    phong_shader->set_uniform("radius", sphere_radius_*1.35f);

    glDepthRange(0.0, 1.0);
    //glPointSize(radius_*1.3f);

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, selection_index_buffer_);
    glDrawElements(GL_POINTS, (GLsizei) point_set_.selected_indices_.size(), GL_UNSIGNED_INT, NULL);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    if (selection_highlight_idx_ != -1)
    {
        phong_shader->set_uniform("front_color",  Vec3f(0.0f,1.0f,0.0f));
        phong_shader->set_uniform("radius", sphere_radius_*2.2f);
        //glPointSize(radius_*2.4f);

        glDrawArrays(GL_POINTS, selection_highlight_idx_, 1);
    }


    glBindVertexArray(0);


}

//-----------------------------------------------------------------------------

void
Point_set_node::
draw_selection_spheres(GL_state* )
{
    if (point_set_.selected_indices_.empty())
        return;

    glDepthRange(0.0, 1.0);


    glVertexAttrib1f(gl::attrib_locations::RADIUS, sphere_radius_ * 1.2f);
    glDisableVertexAttribArray(gl::attrib_locations::COLOR);
    glVertexAttrib3fv(gl::attrib_locations::COLOR, selection_color_.data());

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, selection_index_buffer_);
    glDrawElements(GL_POINTS, (GLsizei) point_set_.selected_indices_.size(), GL_UNSIGNED_INT, NULL);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    if (selection_highlight_idx_ != -1)
    {
        const Vec3f highlight_color = Vec3f(0.0f,1.0f,0.0f);
        glVertexAttrib1f(gl::attrib_locations::RADIUS, sphere_radius_ * 1.4f);
        glVertexAttrib3fv(gl::attrib_locations::COLOR, highlight_color.data());
        glDrawArrays(GL_POINTS, selection_highlight_idx_, 1);
    }


    if (! point_set_.colors_.empty())
    {
        glEnableVertexAttribArray(gl::attrib_locations::COLOR);
    }
    else
    {
        glVertexAttrib3fv(gl::attrib_locations::COLOR, color_.data());
    }

    glBindVertexArray(0);
}

//-----------------------------------------------------------------------------


void
Point_set_node::
apply_transform(const Mat4f &transform)
{
    size_t i;

    for (i=0; i < point_set_.points().size(); ++i)
    {
        Point& p = point_set_.points()[i];
        p = affine_transform(transform, p);
    }

    if (! point_set_.normals_.empty())
    {
        Mat3f normal_mat = inverse(transpose(Mat3f(transform)));
        for (i=0; i < point_set_.normals_.size(); ++i)
        {
            Normal& n = point_set_.normals_[i];
            n = normalize(normal_mat * n);
        }
    }

    update_point_set();
}

//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
