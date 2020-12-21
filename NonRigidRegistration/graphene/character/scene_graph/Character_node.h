//=============================================================================

#ifndef GRAPHENE_CHARACTER_NODE_H
#define GRAPHENE_CHARACTER_NODE_H

//=============================================================================

#include <graphene/surface_mesh/scene_graph/Surface_mesh_node.h>
#include <graphene/character/data_structure/Character.h>

//=============================================================================

namespace graphene {
namespace scene_graph {

//=============================================================================


class Character_node : public Surface_mesh_node
{
    struct Skeleton_gl_data
    {
        GLuint cs_vao_;
        GLuint cs_vertex_buffer_;
        GLuint cs_color_buffer_;

        GLuint bones_vao_;
        GLuint bones_vertex_buffer_;

        GLuint ubo_id_;

        unsigned int n_cs_vertices_;
        unsigned int n_bones_vertices_;


        Skeleton_gl_data() :
            cs_vao_(0),
            cs_vertex_buffer_(0),
            cs_color_buffer_(0),
            bones_vao_(0),
            bones_vertex_buffer_(0),
            ubo_id_(0),
            n_cs_vertices_(0),
            n_bones_vertices_(0)
        {}

        ~Skeleton_gl_data()
        {
            if (cs_vao_)    glDeleteVertexArrays(1,&cs_vao_);
            if (cs_vertex_buffer_)    glDeleteBuffers(1,&cs_vertex_buffer_);
            if (cs_color_buffer_)     glDeleteBuffers(1,&cs_color_buffer_);

            if (bones_vao_) glDeleteVertexArrays(1,&bones_vao_);
            if (bones_vertex_buffer_) glDeleteBuffers(1,&bones_vertex_buffer_);

            if (ubo_id_)              glDeleteBuffers(1,&ubo_id_);
        }
    };

    struct Skin_gl_data
    {
        character::Surface_mesh_skin* skin_;
        character::Blendshapes* blendshapes_;

        GLuint vao_;
        GLuint vertex_buffer_;
        GLuint normal_buffer_;
        GLuint texcoord_buffer_;
        GLuint weight_buffer_;
        GLuint depend_buffer_;
        GLuint weight2_buffer_;
        GLuint depend2_buffer_;
        GLuint color_buffer_;

        GLuint selection_index_buffer_;

        gl::Texture* texture_;
        gl::Texture* texture_nm_;
        gl::Texture* texture_spec_;

        unsigned int n_vertices_;
        unsigned int n_selected_;
        int selection_highlight_idx_;

        Skin_gl_data(character::Surface_mesh_skin* skin) :
            skin_(skin),
            blendshapes_(NULL),
            vao_(0),
            vertex_buffer_(0),
            normal_buffer_(0),
            texcoord_buffer_(0),
            weight_buffer_(0),
            depend_buffer_(0),
            weight2_buffer_(0),
            depend2_buffer_(0),
            color_buffer_(0),
            selection_index_buffer_(0),
            texture_(NULL),
            texture_nm_(NULL),
            texture_spec_(NULL),
            n_vertices_(0),
            n_selected_(0),
            selection_highlight_idx_(-1)
        {}

        ~Skin_gl_data()
        {
            if (vao_) glDeleteVertexArrays(1,&vao_);

            if (vertex_buffer_)   glDeleteBuffers(1,&vertex_buffer_);
            if (normal_buffer_)   glDeleteBuffers(1,&normal_buffer_);
            if (texcoord_buffer_) glDeleteBuffers(1,&texcoord_buffer_);
            if (weight_buffer_)   glDeleteBuffers(1,&weight_buffer_);
            if (depend_buffer_)   glDeleteBuffers(1,&depend_buffer_);
            if (weight2_buffer_)   glDeleteBuffers(1,&weight2_buffer_);
            if (depend2_buffer_)   glDeleteBuffers(1,&depend2_buffer_);
            if (color_buffer_)    glDeleteBuffers(1,&color_buffer_);
        }
    };

private:
    character::Character character_;

    std::vector<Skin_gl_data*> skin_gl_data_;

    Skeleton_gl_data skeleton_gl_data_;

    std::vector<Skin_gl_data*> blendshapes_skin_gl_data_;

    const Vec3f active_mesh_color_;
public:

    Character_node(const std::string& name = "Character_node");
    ~Character_node();

    Surface_mesh& mesh();

    //texture setter/getter: set/get textures of currently selected skin
    void set_texture(gl::Texture *texture, gl::Texture_type type);
    gl::Texture* get_texture(gl::Texture_type type);


    ///currently only transforms base meshes (normals & vertices)
    void apply_transform(const Mat4f& transform);

    character::Character& character() {return character_;}

    bool load(const std::string& filename);
    bool save(const std::string& filename);

    virtual void init(gl::GL_state* gls);

    void add_skin(graphene::character::Surface_mesh_skin* new_skin, gl::GL_state* gls);

    void update_mesh();
    void update_meshes();
    void update_colors();

    void update_blendshapes();

    void set_selected_skin(int selected_skin);
    int set_selected_skin(const std::string& name);

    void delete_selected_skin();

    void apply_blendshapes();

    void scale_to_height(float height, float ground_offset, bool set_on_ground = true, bool no_scale = false);
    void scale(float scale);
    void center();

public:

    void draw(gl::GL_state* _gl);
    void draw_shadowmap(gl::GL_state *gls);

    std::string info() const;

    void clear_selections();
    void invert_selections();
    void select_all();
    void select_isolated();
    void delete_selected();
    void grow_selection();
    void set_selection(const std::vector<size_t>& indices);
    void add_selection(const std::vector<size_t>& indices);
    void get_selection(std::vector<size_t>& indices);
    void clear_selection(const std::vector<size_t>& indices);
    void select_point(Point p);
    void select_landmark(const Point& p);
    void clear_landmarks();
    void highlight_landmark(int idx);
    int  get_num_landmarks();


    void update_skeleton();
    void reset_to_bindpose();

    virtual void update_textures();
    virtual std::vector< gl::Texture* > get_textures() const;

public:

    bool new_data_ = false;

private:

    void init_skin_gl_data(Skin_gl_data* sgd, GL_state* gls);
    void init_skeleton_gl_data();

    void update_skin_points(Skin_gl_data* sgd);
    void update_skin_normals(Skin_gl_data* sgd);
    void update_skin_colors(Skin_gl_data* sgd);
    void update_skin_texcoords(Skin_gl_data* sgd);
    void update_skin_weight_and_depends(Skin_gl_data* sgd);
    void update_skin_texture(Skin_gl_data* sgd);
    void update_skin_selection(Skin_gl_data* sgd);

    void recompute_normals(Skin_gl_data* sgd);

    void draw_selection(gl::GL_state* _gl, Skin_gl_data* sgd);

};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_CHARACTER_NODE_H
//=============================================================================


