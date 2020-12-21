//=============================================================================

#ifndef GRAPHENE_POINT_SET_NODE_H
#define GRAPHENE_POINT_SET_NODE_H

//=============================================================================

#include <graphene/scene_graph/Object_node.h>
#include <graphene/geometry/Point_set.h>

#include <set>

//=============================================================================

namespace graphene {
namespace scene_graph {

//=============================================================================


class Point_set_node : public Object_node
{
public:

    typedef graphene::geometry::Point_set Point_set;


public:

    Point_set_node(const std::string& _name   = "Point Set");
    virtual ~Point_set_node();

    bool load(const std::string& _filename);
    bool save(const std::string& _filename) const;

    void draw(gl::GL_state* _gl);
    Bounding_box bbox() const;
    void update_point_set();

    void apply_transform(const Mat4f& transform);

    void initialize_buffers();

    std::string info() const;

    const float& radius() const { return radius_; }
    void set_radius(const float& radius);

    const Point_set& point_set() const { return point_set_; }
    Point_set& point_set() { return point_set_; }
    void set_point_set(const Point_set& point_set) { point_set_ = point_set; }

    void backup_point_set() { point_set_backup_ = point_set_; }
    const Point_set& get_point_set_backup() const { return point_set_backup_; }

    //selection stuff ---------------------------------------------------------
    void clear_selections();
    void invert_selections();
    void select_all();
    void delete_selected();
    void grow_selection();
    void set_selection(const std::vector<size_t>& indices);
    void add_selection(const std::vector<size_t> &indices);
    void get_selection(std::vector<size_t>& indices);
    void clear_selection(const std::vector<size_t>& indices);
    void select_point(Point p);
    void select_landmark(const Point& p);
    void undo_last_landmark();
    void move_landmark(const int current_landmark_idx, const Point& p);
    void clear_landmarks();
    void highlight_landmark(int idx);
    int  get_num_landmarks();

private:
    void update_selection();

    void draw_selection_points(GL_state* gl_state);
    void draw_selection_spheres(GL_state* gl_state);

protected:

    Point_set    point_set_;
    Point_set    point_set_backup_;
    float        radius_;
    Vec3f        front_color_;
    Vec4f        material_;
    const Vec3f  color_;
    const Vec3f  selection_color_;

    int selection_highlight_idx_;


private:

    // OpenGL buffers
    GLuint vertex_array_object_;
    GLuint vertex_buffer_;
    GLuint normal_buffer_;
    GLuint color_buffer_;

    GLuint selection_index_buffer_;

    float  sphere_radius_;
};


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_POINT_SET_NODE_H
//=============================================================================
