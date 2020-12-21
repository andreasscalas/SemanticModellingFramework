//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/macros.h>
#include <graphene/gl/gl_includes.h>
#include <graphene/gl/surface_mesh_gl.h>
#include <graphene/surface_mesh/scene_graph/Surface_mesh_node.h>
#include <graphene/surface_mesh/data_structure/IO.h>
#include <graphene/utility/Stop_watch.h>

#include <graphene/surface_mesh/scene_graph/mean_curvature_texture.h>
//#include <graphene/surface_mesh/scene_graph/cold_warm_texture.h>

#include <climits>
#include <sstream>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


Surface_mesh_node::
Surface_mesh_node(const std::string& _name)
    : Object_node(_name)
{
    // my geometry object is the Surface_mesh
    this->object_ = &mesh_;


    // init draw modes
    clear_draw_modes();
    add_draw_mode("Points");
    add_draw_mode("Wireframe");
    add_draw_mode("Solid Edge");
    add_draw_mode("Solid");
    add_draw_mode("Scalar Field");
    set_draw_mode(3);


    // init more settings for draw modes
    clear_draw_more_settings();


    // initialize GL buffers to zero
    vertex_array_object_        = 0;
    vertex_buffer_              = 0;
    normal_buffer_              = 0;
    color_buffer_               = 0;
    curvature_texcoord_buffer_  = 0;
    edge_index_buffer_          = 0;
    feature_index_buffer_       = 0;
    selection_index_buffer_     = 0;

    // initialize buffer sizes
    n_vertices_  = 0;
    n_edges_     = 0;
    n_triangles_ = 0;
    n_selected_  = 0;
    n_feature_   = 0;


    skin_subsurface_scattering_ = 0;


//    // initialize texture
    glGenTextures(1, &curvature_texture_);
    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_1D, curvature_texture_);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, mean_curvature_texture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    selection_highlight_idx_ = -1;

    // material parameters
    front_color_  = Vec3f(0.4, 0.425, 0.475);
    back_color_   = Vec3f(0.5, 0.3, 0.3);
    wire_color_   = Vec3f(0,0,0);
    material_     = Vec4f(0.1, 1.0, 1.0, 100.0);
    crease_angle_ = 0.0;
    texture_      = NULL;
    texture_nm_   = NULL;
    texture_spec_ = NULL;


    // selection modes
    clear_selection_modes();
    add_selection_mode("Lasso");
    add_selection_mode("Constraints");
    add_selection_mode("Landmarks");
    set_selection_mode(0);
}


//-----------------------------------------------------------------------------


Surface_mesh_node::
~Surface_mesh_node()
{
    if (skin_subsurface_scattering_) delete skin_subsurface_scattering_;

    // delete texture
    glDeleteTextures(1, &curvature_texture_);

    // delete all OpenGL buffers
    delete_buffers();
}


//-----------------------------------------------------------------------------


bool
Surface_mesh_node::
load(const std::string& filename)
{
    bool read = false;
    fileinfo_ = filename;

    mesh_.clear();
    read = mesh_.read(filename);

    LOG(Log_info) << mesh_.n_vertices() << " Vertices, "
                  << mesh_.n_faces()    << " Faces." << std::endl;

    update_mesh();

    return read;
}


//-----------------------------------------------------------------------------


bool
Surface_mesh_node::
save(const std::string& filename) const
{
    return mesh_.write(filename);
}


//-----------------------------------------------------------------------------


std::string
Surface_mesh_node::
info() const
{
    std::ostringstream s;
    s << mesh_.n_vertices() << " vertices, "
      << mesh_.n_edges() << " edges, "
      << mesh_.n_faces() << " faces";
    return s.str();
}


//-----------------------------------------------------------------------------

void
Surface_mesh_node::
init(gl::GL_state *_gl)
{

    Surface_mesh::Mesh_property<std::string> texname = mesh_.get_mesh_property<std::string>("m:texturename");
    if (texname && !texname.vector().empty())
    {
        gl::Texture* texture = _gl->get_texture_from_heap(texname[0]);
        if (texture != nullptr)
        {
            set_texture(texture, gl::TT_COLOR);
        }
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
draw(gl::GL_state *gl)
{
    if (!visible_)
        return;

    // upload mesh data, if not done already
    if (!vertex_array_object_)
    {
        initialize_buffers();
    }

    Vec3f front_color = front_color_;
    if (is_target_)
        front_color *= Vec3f(0.8f,1.0f,0.8f);

    // compute matrices
    gl->update_matrices();


    // setup Phong shader
    gl::Shader* phong_shader = gl->set_active_shader(gl::PHONG_SHADER);
    phong_shader->use();
    phong_shader->set_uniform("use_lighting",    false);
    phong_shader->set_uniform("use_vertexcolor", false);
    phong_shader->set_uniform("use_texture1D",     false);
    phong_shader->set_uniform("use_texture2D",     false);
    phong_shader->set_uniform("material",     material_);
    phong_shader->set_uniform("front_color",  front_color_);
    phong_shader->set_uniform("back_color",   back_color_);
    phong_shader->set_uniform("model_matrix", gl->model_);
    phong_shader->set_uniform("modelview_projection_matrix", gl->modelviewproj_);
    phong_shader->set_uniform("modelview_matrix", gl->modelview_);
    phong_shader->set_uniform("normal_matrix", gl->normal_);
    phong_shader->set_uniform("use_normalmap", false);
    phong_shader->set_uniform("use_specularmap", false);


    // shadow map stuff

    graphene::gl::Shadow_map* sm;

    if (gl->use_shadowmaps_)
    {
        phong_shader->set_uniform("use_shadowmaps", true);

        phong_shader->set_uniform("strength", 0.75f);

        if (gl->get_shadow_maps().size() > 0)
        {
            sm = gl->get_shadow_maps()[0];
            phong_shader->bind_texture("shadowmap0", sm->get_depth_texture_id(), GL_TEXTURE_2D, 4);
            phong_shader->set_uniform("viewproj0", sm->viewproj_);
        }

        if (gl->get_shadow_maps().size() > 1)
        {
            sm = gl->get_shadow_maps()[1];
            phong_shader->bind_texture("shadowmap1", sm->get_depth_texture_id(), GL_TEXTURE_2D, 5);
            phong_shader->set_uniform("viewproj1", sm->viewproj_);
        }
    }
    else
    {
        phong_shader->set_uniform("use_shadowmaps", false);
    }


    // get current draw mode
    std::string draw_mode = get_draw_mode();

    // get current active settings for draw mode
    std::vector<std::string> draw_more_settings = get_draw_more_settings_active();


    // VAO init
    glBindVertexArray(vertex_array_object_);


    if (draw_mode == "Wireframe")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        phong_shader->set_uniform("use_lighting", false);
        phong_shader->set_uniform("use_vertexcolor", false);
        phong_shader->set_uniform("front_color", wire_color_);

        glDepthRange(0.001, 1.0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_index_buffer_);
        glDrawElements(GL_LINES, n_edges_, GL_UNSIGNED_INT, NULL);
    }


    else if (draw_mode == "Solid Edge")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        // solid
        phong_shader->set_uniform("use_lighting", true);
        phong_shader->set_uniform("use_vertexcolor", has_colors());
        glDepthRange(0.002, 1.0);
        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);


        // edges
        phong_shader->set_uniform("use_lighting", false);
        phong_shader->set_uniform("front_color", wire_color_);
        phong_shader->set_uniform("use_vertexcolor", false);
        glDepthRange(0.001, 1.0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_index_buffer_);
        glDrawElements(GL_LINES, n_edges_, GL_UNSIGNED_INT, NULL);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }


    else if (draw_mode == "Solid")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        phong_shader->set_uniform("use_lighting", true);
        phong_shader->set_uniform("use_vertexcolor", has_colors());
        glDepthRange(0.002, 1.0);

        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
    }


    else if (draw_mode == "Points")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        phong_shader->set_uniform("use_lighting", true);
        glDepthRange(0.0, 1.0);
        glPointSize(3.0f);
        glDrawArrays(GL_POINTS, 0, n_vertices_);
    }


    else if (draw_mode == "Scalar Field")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        glDepthRange(0.002, 1.0);
        phong_shader->set_uniform("use_texture1D", true);
        phong_shader->set_uniform("use_lighting", true);
        gl->bind_texture("texture1D", curvature_texture_, GL_TEXTURE_1D, 1);
        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
    }

    else if (draw_mode == "Textured (Shaded)")
    {
        bool with_normal_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Normal Map") != draw_more_settings.end())
        {
            with_normal_map = true;
        }

        bool with_specular_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Specular Map") != draw_more_settings.end())
        {
            with_specular_map = true;
        }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        if (texture_)
        {
            glBindBuffer(GL_ARRAY_BUFFER, diffuse_texcoord_buffer_);
            glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 3, GL_FLOAT, GL_FALSE, 0, 0);

            phong_shader->set_uniform("use_texture2D", true);
            phong_shader->bind_texture("texture2D", texture_->id_, GL_TEXTURE_2D, 1);
            phong_shader->set_uniform("front_color", Vec3f(1.0f));
            phong_shader->set_uniform("use_lighting", true);
        }
        else
        {
            phong_shader->set_uniform("front_color", Vec3f(0.5f,0.0f,0.0f));
        }

        if (with_normal_map)
        {
            phong_shader->bind_texture("texture2D_nm", texture_nm_->id_, GL_TEXTURE_2D, 2);
            phong_shader->set_uniform("use_normalmap", true);
        }
        else
        {
            phong_shader->set_uniform("use_normalmap", false);
        }

        if (with_specular_map)
        {
            phong_shader->bind_texture("texture2D_spec", texture_spec_->id_, GL_TEXTURE_2D, 3);
            phong_shader->set_uniform("use_specularmap", true);
        }
        else
        {
            phong_shader->set_uniform("use_specularmap", false);
        }

        glDepthRange(0.002, 1.0);

        glEnable(GL_FRAMEBUFFER_SRGB);
        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
        glDisable(GL_FRAMEBUFFER_SRGB);
    }

    else if (draw_mode == "Textured")
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        if (texture_)
        {
            glBindBuffer(GL_ARRAY_BUFFER, diffuse_texcoord_buffer_);
            glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 3, GL_FLOAT, GL_FALSE, 0, 0);

            phong_shader->set_uniform("use_texture2D", true);
            phong_shader->bind_texture("texture2D", texture_->id_, GL_TEXTURE_2D, 1);
            phong_shader->set_uniform("front_color", Vec3f(1.0f));
        }
        else
        {
            phong_shader->set_uniform("front_color", Vec3f(0.5f,0.0f,0.0f));
        }


        glDepthRange(0.002, 1.0);

        glEnable(GL_FRAMEBUFFER_SRGB);
        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
        glDisable(GL_FRAMEBUFFER_SRGB);
    }

    else if (draw_mode == "Skin Shader")
    {
        bool with_normal_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Normal Map") != draw_more_settings.end())
        {
            with_normal_map = true;
        }

        bool with_specular_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Specular Map") != draw_more_settings.end())
        {
            with_specular_map = true;
        }


        if (!skin_subsurface_scattering_)
        {
            // initialize
            skin_subsurface_scattering_ =
                new gl::Skin_subsurface_scattering(gl,
                                                   vertex_array_object_,
                                                   n_vertices_,
                                                   texture_,
                                                   texture_nm_,
                                                   texture_spec_);
        }

        const float mixRatio = 0.5f; // 1 means full pre-scattering; 0 means full post-scattering; 0.5 is a good tradeoff

        const float factor_blurring = 0.05f; // factor blurring

        skin_subsurface_scattering_->apply(mixRatio,
                                           factor_blurring,
                                           material_,
                                           front_color_,
                                           back_color_,
                                           with_normal_map,
                                           with_specular_map);
    }


    // draw selected vertices
    if (n_selected_)
    {
        glDepthRange(0.0, 1.0);
        glPointSize(5.0);

        phong_shader->use();

        phong_shader->set_uniform("use_lighting",    true);
        phong_shader->set_uniform("use_vertexcolor", false);
        phong_shader->set_uniform("use_texture1D", false);
        phong_shader->set_uniform("use_texture2D", false);
        phong_shader->set_uniform("front_color",     Vec3f(1.0f, 0.0f, 0.0f));
        phong_shader->set_uniform("material",        Vec4f(0.1, 1.0, 0.0, 0.0));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, selection_index_buffer_);
        glDrawElements(GL_POINTS, n_selected_, GL_UNSIGNED_INT, NULL);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        if (selection_highlight_idx_ != -1)
        {
            glPointSize(10.0);
            phong_shader->set_uniform("front_color",     Vec3f(0.0f, 1.0f, 0.0f));
            glDrawArrays(GL_POINTS, selection_highlight_idx_, 1);
        }
    }


    // draw feature edges
    if (n_feature_)
    {
        phong_shader->use();

        phong_shader->set_uniform("use_lighting",    false);
        phong_shader->set_uniform("use_vertexcolor", false);
        phong_shader->set_uniform("use_texture1D", false);
        phong_shader->set_uniform("use_texture2D", false);
        phong_shader->set_uniform("front_color",     Vec3f(0.961f, 0.624f, 0.0f));
        phong_shader->set_uniform("material",        Vec4f(0.1, 1.0, 0.0, 0.0));

        glDepthRange(0.0, 1.0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, feature_index_buffer_);
        glDrawElements(GL_LINES, n_feature_, GL_UNSIGNED_INT, NULL);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }


    // reset stuff
    glBindVertexArray(0);
    glPointSize(1.0);
    glDepthRange(0.0, 1.0);
    phong_shader->disable();

}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
draw_shadowmap(gl::GL_state *gls)
{
    gl::Shader* shader = gls->set_active_shader(gl::MESH_SHADOWMAP_SHADER);
    shader->set_uniform("modelview_projection_matrix", gls->modelviewproj_);

    glBindVertexArray(vertex_array_object_);
    glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
    glBindVertexArray(0);
}


//-----------------------------------------------------------------------------


geometry::Bounding_box
Surface_mesh_node::
bbox() const
{
    return bbox_;
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_mesh()
{
    //utility::Stop_watch timer; timer.start();


    // generate buffers
    if (!vertex_array_object_)
    {
        initialize_buffers();
    }


    // activate VAO
    glBindVertexArray(vertex_array_object_);


    // data that we will eventually duplicate
    std::vector<Point>        points;
    std::vector<Normal>       normals;

    points.reserve(mesh_.n_faces() * 3);
    normals.reserve(mesh_.n_faces() * 3);

    auto vpoints        = mesh_.vertex_property<Point>("v:point");
    auto vertex_indices = mesh_.vertex_property<size_t>("v:index");

    size_t i(0);
    for (auto f : mesh_.faces())
    {
        Surface_mesh::Vertex_around_face_circulator fvit, fvend;
        Surface_mesh::Vertex v0, v1, v2;

        fvit = fvend = mesh_.vertices(f);
        v0 = *fvit; ++fvit;
        v2 = *fvit; ++fvit;
        do
        {
            v1 = v2;
            v2 = *fvit;

            points.push_back(vpoints[v0]);
            points.push_back(vpoints[v1]);
            points.push_back(vpoints[v2]);

            vertex_indices[v0] = i++;
            vertex_indices[v1] = i++;
            vertex_indices[v2] = i++;
        }
        while (++fvit != fvend);
    }



    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, points.size()*3*sizeof(float), points.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);
    n_vertices_ = points.size();


    // normals
    crease_normals(normals);
    glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
    glBufferData(GL_ARRAY_BUFFER, normals.size()*3*sizeof(float), normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::NORMAL, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::NORMAL);

    // edge indices
    std::vector<unsigned int> edges;
    edges.reserve(mesh_.n_edges());
    for (auto e : mesh_.edges())
    {
        edges.push_back(vertex_indices[mesh_.vertex(e, 0)]);
        edges.push_back(vertex_indices[mesh_.vertex(e, 1)]);
    }
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_index_buffer_);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, edges.size()*sizeof(unsigned int), &edges[0], GL_STATIC_DRAW);
    n_edges_ = edges.size();



    // upload either per-vertex or per-face colors
    update_colors();

    // update texture coordinates
    update_texcoords();

    // collect feature edges for rendering (careful: disables VAO)
    update_features();

    // collect selected vertices (careful: disables VAO)
    update_selection();

    // recompute bounding box
    update_bbox();

    //glBindVertexArray(0);

    //timer.stop();
    //LOG(Log_debug) << "Update mesh took " << timer << std::endl;
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_colors()
{
    if (!has_colors())
    {
        glBindVertexArray(vertex_array_object_);
        glDisableVertexAttribArray(gl::attrib_locations::COLOR);
        return;
    }

    std::vector<Color> colors;
    colors.reserve(mesh().n_faces() * 3);

    auto vcolors = mesh().get_vertex_property<Color>("v:color");
    auto fcolors = mesh().get_face_property<Color>("f:color");

    for (auto f : mesh().faces())
    {
        Surface_mesh::Vertex_around_face_circulator fvit, fvend;
        Surface_mesh::Vertex v0, v1, v2;

        fvit = fvend = mesh().vertices(f);
        v0 = *fvit; ++fvit;
        v2 = *fvit; ++fvit;
        do
        {
            v1 = v2;
            v2 = *fvit;

            if (vcolors)
            {
                colors.push_back(vcolors[v0]);
                colors.push_back(vcolors[v1]);
                colors.push_back(vcolors[v2]);
            }

            if (fcolors)
            {
                colors.push_back(fcolors[f]);
                colors.push_back(fcolors[f]);
                colors.push_back(fcolors[f]);
            }
        }
        while (++fvit != fvend);
    }

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, colors.size()*3*sizeof(float), colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::COLOR, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::COLOR);
    glBindVertexArray(0);
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_texcoords()
{
    auto vtexcoords = mesh_.get_vertex_property<float>("v:tex_1D");
    auto htexcoords = mesh_.get_halfedge_property<Texture_coordinate>("h:texcoord");

    if (!vtexcoords && !htexcoords)
    {
        glBindVertexArray(vertex_array_object_);
        glDisableVertexAttribArray(gl::attrib_locations::TEXCOORDS);
        return;
    }

    std::vector<float> texcoords1D;
    std::vector<Texture_coordinate> texcoords2D;
    texcoords1D.reserve(mesh_.n_faces() * 3);
    texcoords2D.reserve(mesh_.n_faces() * 3);
    Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    Surface_mesh::Halfedge_around_face_circulator hfc_it,hfc_end;
    Surface_mesh::Vertex v0, v1, v2;


    for (auto f : mesh_.faces())
    {

        if (vtexcoords)
        {
            fvit = fvend = mesh_.vertices(f);
            v0 = *fvit; ++fvit;
            v2 = *fvit; ++fvit;
            do
            {
                v1 = v2;
                v2 = *fvit;

                texcoords1D.push_back((vtexcoords[v0]));
                texcoords1D.push_back((vtexcoords[v1]));
                texcoords1D.push_back((vtexcoords[v2]));
            }
            while (++fvit != fvend);
        }

        if (htexcoords)
        {
            //twaltema: gather texture coords for each generated/duplicated vertex from halfedge properties
            //ATTENTION: this does not work for quad meshes
            hfc_it = hfc_end = mesh_.halfedges(f);
            do
            {
                texcoords2D.push_back(htexcoords[*hfc_it]);
            }
            while (++hfc_it != hfc_end);
        }
    }

    glBindVertexArray(vertex_array_object_);

    if (!texcoords1D.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, curvature_texcoord_buffer_);
        glBufferData(GL_ARRAY_BUFFER, texcoords1D.size()*sizeof(float), texcoords1D.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 1, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(gl::attrib_locations::TEXCOORDS);
    }

    if (!texcoords2D.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, diffuse_texcoord_buffer_);
        glBufferData(GL_ARRAY_BUFFER, texcoords2D.size()*sizeof(float)*3, texcoords2D.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(gl::attrib_locations::TEXCOORDS);
    }

    glBindVertexArray(0);
}



//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_features()
{
    std::vector<unsigned int> features;
    features.reserve(2*mesh_.n_edges());

    auto vertex_indices = mesh_.vertex_property<size_t>("v:index");
    auto efeature       = mesh_.get_edge_property<bool>("e:feature");

    if (efeature)
    {
        for (auto e: mesh_.edges())
        {
            if (efeature[e])
            {
                features.push_back(vertex_indices[mesh_.vertex(e, 0)]);
                features.push_back(vertex_indices[mesh_.vertex(e, 1)]);
            }
        }
    }

    n_feature_ = features.size();
    if (n_feature_ == 0){ return;} //otherwise it will crash on Windows

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, feature_index_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, features.size()*sizeof(unsigned int), &features[0], GL_STATIC_DRAW);
    glBindVertexArray(0);
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_selection()
{
    std::vector<unsigned int> selection;

    auto vertex_indices = mesh_.vertex_property<size_t>("v:index");
    auto vselected      = mesh_.get_vertex_property<bool>("v:selected");

    if (vselected)
    {
        for (auto v: mesh_.vertices())
        {
            if (vselected[v])
            {
                selection.push_back(vertex_indices[v]);
            }
        }
    }

    n_selected_ = selection.size();
    if (n_selected_ == 0){ return;} //otherwise it will crash on Windows

    glBindVertexArray(vertex_array_object_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, selection_index_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, selection.size()*sizeof(unsigned int), &selection[0], GL_STATIC_DRAW);
    glBindVertexArray(0);
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
update_bbox()
{
    auto points   = mesh_.vertex_property<Point>("v:point");

    bbox_ = Bounding_box();
    for (auto v : mesh_.vertices())
    {
        bbox_ += points[v];
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
select_point(Point p)
{
    Surface_mesh::Vertex v_min;
    Scalar  d, d_min(FLT_MAX);

    auto points   = mesh_.vertex_property<Point>("v:point");
    auto selected = mesh_.vertex_property<bool>("v:selected");

    for (auto v: mesh_.vertices())
    {
        d = distance(points[v], (Point)p);
        if (d < d_min)
        {
            v_min = v;
            d_min = d;
        }
    }
    std::cerr << "[DEBUG]: idx -> " << v_min.idx() << std::endl;
    selected[v_min] = true;
    update_selection();
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
select_landmark(const Point& p)
{
    Surface_mesh::Vertex v_min;
    Scalar  d, d_min(FLT_MAX);

    auto points   = mesh_.vertex_property<Point>("v:point");
    auto selected = mesh_.vertex_property<bool>("v:selected");
    auto landmarks = mesh_.mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

    auto landmarks_lw = mesh_.get_vertex_property< double >("v:landmark_weight");

    for (auto v: mesh_.vertices())
    {
        d = distance(points[v], (Point)p);
        if (d < d_min)
        {
            v_min = v;
            d_min = d;
        }
    }
    selected[v_min] = true;
    lm.push_back(v_min);
    if (landmarks_lw)
    {
        landmarks_lw[v_min] = 1.0;
    }
    update_selection();
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
undo_last_landmark()
{
    Surface_mesh::Vertex v;

    auto selected = mesh().vertex_property<bool>("v:selected");
    auto landmarks = mesh().mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

    if (lm.empty())
        return;

    v = lm.back();
    lm.pop_back();

    auto landmarks_lw = mesh().get_vertex_property< double >("v:landmark_weight");

    selected[v] = false;
    if (landmarks_lw)
    {
        landmarks_lw[v] = 0.0;
    }
    update_selection();
}

//-----------------------------------------------------------------------------


void
Surface_mesh_node::
clear_landmarks()
{
    auto landmarks = mesh_.get_mesh_property< std::vector<Surface_mesh::Vertex> > ("m:landmarks");
    auto landmarks_lw = mesh().get_vertex_property< double > ("v:landmark_weight");
    if (landmarks)
    {
        mesh_.remove_mesh_property(landmarks);
    }
    if (landmarks_lw)
    {
        mesh_.remove_vertex_property(landmarks_lw);
    }
}

//-----------------------------------------------------------------------------


void
Surface_mesh_node::
highlight_landmark(int idx)
{
    Surface_mesh::Mesh_property< std::vector< Surface_mesh::Vertex > > landmarks = mesh_.get_mesh_property< std::vector< Surface_mesh::Vertex > >("m:landmarks");

    if (! landmarks)
    {
        selection_highlight_idx_ = -1;
        return;
    }

    std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

    Surface_mesh::Vertex v = lm[idx];
    Surface_mesh::Vertex_property<size_t> vindices = mesh_.get_vertex_property<size_t>("v:index");
    if (vindices && idx != -1)
    {
        selection_highlight_idx_ = (int) vindices[v];
    }
    else
    {
        selection_highlight_idx_ = -1;
    }

    update_selection();
}


//-----------------------------------------------------------------------------


int
Surface_mesh_node::
get_num_landmarks()
{
    Surface_mesh::Mesh_property< std::vector< Surface_mesh::Vertex > > landmarks = mesh().get_mesh_property< std::vector< Surface_mesh::Vertex > >("m:landmarks");
    if (landmarks)
    {
        std::vector<Surface_mesh::Vertex> &lm = landmarks[0];
        if (lm.empty())
        {
            selection_highlight_idx_ = -1;
            return 0;
        }
        else
        {
            return (int) lm.size();
        }
    }
    else
    {
        selection_highlight_idx_ = -1;
        return 0;
    }
}

//-----------------------------------------------------------------------------


void
Surface_mesh_node::
clear_selections()
{
    auto selected = mesh_.get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh_.vertices())
        {
            selected[v] = false;
        }
        update_selection();

        mesh_.remove_vertex_property(selected);
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
invert_selections()
{
    auto selected = mesh_.get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh_.vertices())
        {
            selected[v] = !selected[v];
        }
        update_selection();
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
select_all()
{
    auto selected = mesh_.vertex_property<bool>("v:selected");

    for (auto v: mesh_.vertices())
    {
        selected[v] = true;
    }
    update_selection();
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
select_isolated()
{
    for (auto v: mesh_.vertices())
    {
        if (mesh_.is_isolated(v))
        {
            auto selected = mesh_.vertex_property<bool>("v:selected");
            selected[v] = true;
        }
    }
    if (mesh_.get_vertex_property<bool>("v:selected"))
        update_selection();
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
set_selection(const std::vector<size_t>& indices)
{
    auto selected = mesh_.vertex_property<bool>("v:selected");

    for (size_t i=0; i < selected.vector().size(); i++)
    {
        selected.vector()[i] = false;
    }

    for (size_t i(0); i < indices.size(); i++)
    {
        selected[Surface_mesh::Vertex(indices[i])] = true;
    }
    update_selection();
}


//-----------------------------------------------------------------------------

void
Surface_mesh_node::
add_selection(const std::vector<size_t> &indices)
{
    auto selected = mesh_.vertex_property<bool>("v:selected");

    for (size_t i(0); i < indices.size(); i++)
    {
        selected[Surface_mesh::Vertex(indices[i])] = true;
    }
    update_selection();
}

//-----------------------------------------------------------------------------


void
Surface_mesh_node::
delete_selected()
{
    auto selected = mesh_.get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh_.vertices())
        {
            if (selected[v])
            {
                selected[v] = false;
                mesh_.delete_vertex(v);
            }
        }
        mesh_.garbage_collection();
        update_mesh();
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
get_selection(std::vector<size_t>& indices)
{
    indices.clear();
    auto selected = mesh_.get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh_.vertices())
        {
            if (selected[v])
            {
                indices.push_back(v.idx());
            }
        }
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
clear_selection(const std::vector<size_t>& indices)
{
    auto selected = mesh_.get_vertex_property<bool>("v:selected");
    if (selected)
    {
        for (auto i : indices)
        {
            selected[Surface_mesh::Vertex(i)] = false;
        }
    }
    update_selection();
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
grow_selection()
{
    std::vector<Surface_mesh::Vertex> vertices;
    auto selected = mesh_.get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v : mesh_.vertices())
        {
            if (selected[v])
            {
                for (auto vv: mesh_.vertices(v))
                {
                    vertices.push_back(vv);
                }
            }
        }

        for (auto v : vertices)
            selected[v] = true;

        update_selection();
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
initialize_buffers()
{
    if (vertex_array_object_ == 0)
    {
        glGenVertexArrays(1, &vertex_array_object_);
        glBindVertexArray(vertex_array_object_);

        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &normal_buffer_);
        glGenBuffers(1, &color_buffer_);
        glGenBuffers(1, &curvature_texcoord_buffer_);
        glGenBuffers(1, &diffuse_texcoord_buffer_);
        glGenBuffers(1, &edge_index_buffer_);
        glGenBuffers(1, &feature_index_buffer_);
        glGenBuffers(1, &selection_index_buffer_);

        glBindVertexArray(0);
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
delete_buffers()
{
    if (vertex_buffer_)             glDeleteBuffers(1, &vertex_buffer_);
    if (normal_buffer_)             glDeleteBuffers(1, &normal_buffer_);
    if (color_buffer_)              glDeleteBuffers(1, &color_buffer_);
    if (curvature_texcoord_buffer_) glDeleteBuffers(1, &curvature_texcoord_buffer_);

    if (edge_index_buffer_)         glDeleteBuffers(1, &edge_index_buffer_);
    if (feature_index_buffer_)      glDeleteBuffers(1, &feature_index_buffer_);
    if (selection_index_buffer_)    glDeleteBuffers(1, &selection_index_buffer_);

    if (vertex_array_object_)       glDeleteVertexArrays(1, &vertex_array_object_);
}



//-----------------------------------------------------------------------------


void
Surface_mesh_node::
crease_normals(std::vector<Normal>& vertex_normals)
{
    bool is_triangle = mesh_.is_triangle_mesh();
    auto points = mesh_.vertex_property<Point>("v:point");

    if (crease_angle_ != 0 && is_triangle)
    {
        const Scalar crease = cos( crease_angle_ / 180.0 * M_PI );
        Normal n, ni, nni;
        Surface_mesh::Vertex v1, v2;
        Point e1, e2;
        Scalar w;

        mesh_.update_face_normals();
        auto fnormals = mesh_.face_property<Normal>("f:normal");

        for (auto f : mesh_.faces())
        {
            ni = fnormals[f];

            for (auto v : mesh_.vertices(f))
            {
                n = 0.0;

                for (auto h : mesh_.halfedges(v))
                {
                    if (!mesh_.is_boundary(h))
                    {
                        nni = fnormals[ mesh_.face(h) ];

                        if (dot(ni,nni) > crease)
                        {
                            v1  = mesh_.to_vertex(h);
                            v2  = mesh_.to_vertex(mesh_.prev_halfedge(mesh_.prev_halfedge(h)));
                            e1  = (points[v1] - points[v]).normalize();
                            e2  = (points[v2] - points[v]).normalize();
                            w   = acos( std::max(-1.0f, std::min(1.0f, dot(e1, e2) )));
                            n  += w * nni;
                        }
                    }
                }

                vertex_normals.push_back(n.normalize());
            }
        }
    }
    else
    {
        if (is_triangle)
            mesh_.update_vertex_normals();
        else
            mesh_.update_face_normals();

        auto vnormals = mesh_.get_vertex_property<Normal>("v:normal");
        auto fnormals = mesh_.face_property<Normal>("f:normal");

        for (auto f : mesh_.faces())
        {
            Surface_mesh::Vertex_around_face_circulator fvit, fvend;
            Surface_mesh::Vertex v0, v1, v2;

            fvit = fvend = mesh_.vertices(f);
            v0 = *fvit; ++fvit;
            v2 = *fvit; ++fvit;
            do
            {
                v1 = v2;
                v2 = *fvit;

                if (is_triangle)
                {
                    vertex_normals.push_back(vnormals[v0]);
                    vertex_normals.push_back(vnormals[v1]);
                    vertex_normals.push_back(vnormals[v2]);
                }
                else
                {
                    vertex_normals.push_back(fnormals[f]);
                    vertex_normals.push_back(fnormals[f]);
                    vertex_normals.push_back(fnormals[f]);
                }
            }
            while (++fvit != fvend);
        }
    }
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
set_texture(gl::Texture *texture, gl::Texture_type type)
{

    switch (type)
    {
    case gl::TT_COLOR:
        texture_ = texture;
        break;
    case gl::TT_NORMAL:
        texture_nm_ = texture;
        break;
    case gl::TT_SPECULAR:
        texture_spec_ = texture;
        break;
    default:
        break;
    }

    update_textures();
}
//-----------------------------------------------------------------------------


gl::Texture*
Surface_mesh_node::
get_texture(gl::Texture_type type)
{

    switch (type)
    {
    case gl::TT_COLOR:
        return texture_;
        break;
    case gl::TT_NORMAL:
        return texture_nm_;
        break;
    case gl::TT_SPECULAR:
        return texture_spec_;
        break;
    default:
        break;
    }
}


void
Surface_mesh_node::
update_textures()
{
    if (texture_)
    {
        if (texture_->id_ == 0)
        {
            glGenTextures(1, &texture_->id_);
        }

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_->id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, texture_->filter_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, texture_->filter_);
        if (texture_->srgb_)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, texture_->width_, texture_->height_,
                         0, GL_RGBA, GL_UNSIGNED_BYTE, texture_->data_.data());
        }
        else
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texture_->width_, texture_->height_,
                         0, GL_RGBA, GL_UNSIGNED_BYTE, texture_->data_.data());
        }

        add_draw_mode("Textured (Shaded)");
        set_draw_mode("Textured (Shaded)");
        add_draw_mode("Textured");
        add_draw_mode("Skin Shader");


        material_     = Vec4f(0.0, 0.6, 0.1, 64.0);
    }

    if (texture_ && texture_nm_ && texture_spec_)
    {
        if (texture_nm_->id_ == 0) // normal map
        {
            glGenTextures(1, &texture_nm_->id_);
        }

        // normal map
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texture_nm_->id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_nm_->width_, texture_nm_->height_,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, texture_nm_->data_.data());

        if (texture_spec_->id_ == 0) // normal map
        {
            glGenTextures(1, &texture_spec_->id_);
        }

        // specular map
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, texture_spec_->id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_spec_->width_, texture_spec_->height_,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, texture_spec_->data_.data());

        add_draw_more_settings("Normal Map");
        add_draw_more_settings("Specular Map");
    }
}


//-----------------------------------------------------------------------------


std::vector< gl::Texture* >
Surface_mesh_node::
get_textures() const
{
    std::vector< gl::Texture* > textures;

    if (texture_)
    {
        textures.push_back(texture_);
    }

    if (texture_nm_)
    {
        textures.push_back(texture_nm_);
    }

    if (texture_spec_)
    {
        textures.push_back(texture_spec_);
    }

    return textures;
}


//-----------------------------------------------------------------------------


void
Surface_mesh_node::
apply_transform(const Mat4f &transform)
{
    Surface_mesh::Vertex_property<Point> vpoints = mesh_.get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Normal> vnormals = mesh_.get_vertex_property<Normal>("v:normal");
    Surface_mesh::Vertex_iterator v_it;

    if (vpoints)
    {
        for (v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it)
        {
            Point& p = vpoints[*v_it];
            p = affine_transform(transform, p);
        }
    }

    if (vnormals)
    {
        Mat3f normal_mat = inverse(transpose(Mat3f(transform)));

        for (v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it)
        {
            Normal &n = vnormals[*v_it];
            n = normal_mat * n;
        }
    }

    update_mesh();
}


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
