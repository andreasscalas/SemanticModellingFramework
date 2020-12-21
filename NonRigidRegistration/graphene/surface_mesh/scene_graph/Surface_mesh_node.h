//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_SURFACE_MESH_NODE_H
#define GRAPHENE_SURFACE_MESH_NODE_H
//=============================================================================


#include <graphene/scene_graph/Object_node.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/gl/skin_subsurface_scattering/Skin_subsurface_scattering.h>
#include <graphene/gl/Shader.h>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


/// \addtogroup scene_graph
/// @{


/// A scene graph node for redering surface meshes
class Surface_mesh_node : public Object_node
{

public:

    typedef ::graphene::surface_mesh::Surface_mesh Surface_mesh;

    Surface_mesh_node(const std::string& _name   = "Surface_mesh_node");
    virtual ~Surface_mesh_node();

    bool load(const std::string& filename);
    virtual bool save(const std::string& filename) const;

    void init(gl::GL_state* _gl);

    void draw(gl::GL_state* _gl);
    void draw_shadowmap(gl::GL_state *gls);

    Bounding_box bbox() const;
    virtual std::string info() const;

    void clear_selections();
    void invert_selections();
    void select_all();
    void select_isolated();
    void delete_selected();
    void grow_selection();
    void set_selection(const std::vector<size_t>& indices);
    void add_selection(const std::vector<size_t> &indices);
    void get_selection(std::vector<size_t>& indices);
    void clear_selection(const std::vector<size_t>& indices);
    void select_point(Point p);
    void select_landmark(const Point& p);
    void undo_last_landmark();
    virtual void clear_landmarks();
    virtual void highlight_landmark(int idx);
    virtual int  get_num_landmarks();

    virtual void update_mesh();
    virtual void update_colors();
    void update_texcoords();
    void update_features();
    void update_selection();
    void update_bbox();
    void crease_normals(std::vector<Normal>& vertex_normals);

    //twaltema: set (diffuse) texture for this mesh; this adds/removes textured draw mode
    virtual void set_texture(gl::Texture* texture, gl::Texture_type type);
    virtual gl::Texture* get_texture(gl::Texture_type type);


    virtual void update_textures();
    virtual std::vector< gl::Texture* > get_textures() const;

    virtual bool has_colors()
    {
        return mesh().get_vertex_property<Color>("v:color") ||
            mesh().get_face_property<Color>("f:color");
    }

    virtual void apply_transform(const Mat4f& transform);

private:
    Surface_mesh mesh_;

    int selection_highlight_idx_;
public:

    // material
    Vec3f  front_color_;
    Vec3f  back_color_;
    Vec3f  wire_color_;
    Vec4f  material_;
    double crease_angle_;

    virtual Surface_mesh& mesh() {return mesh_;}

private:

    void initialize_buffers();
    void delete_buffers();


private:

    // OpenGL buffers
    GLuint vertex_array_object_;
    GLuint vertex_buffer_;
    GLuint normal_buffer_;
    GLuint color_buffer_;
    GLuint curvature_texcoord_buffer_;
    GLuint diffuse_texcoord_buffer_;
    GLuint edge_index_buffer_;
    GLuint feature_index_buffer_;
    GLuint selection_index_buffer_;
    GLuint curvature_texture_;

    gl::Texture* texture_;
    gl::Texture* texture_nm_;
    gl::Texture* texture_spec_;

    gl::Skin_subsurface_scattering* skin_subsurface_scattering_;

    // buffer sizes
    GLsizei n_vertices_;
    GLsizei n_edges_;
    GLsizei n_triangles_;
    GLsizei n_selected_;
    GLsizei n_feature_;
};


//=============================================================================
/// @}
//=============================================================================
} // namespace scnene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SURFACE_MESH_NODE_H
//=============================================================================
