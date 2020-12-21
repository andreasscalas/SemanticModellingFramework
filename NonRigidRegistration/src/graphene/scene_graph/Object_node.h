//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_OBJECT_NODE_H
#define GRAPHENE_OBJECT_NODE_H
//=============================================================================


#include <graphene/scene_graph/Base_node.h>
#include <graphene/geometry/Geometry_representation.h>
#include <graphene/gl/GL_state.h>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


/// This is the base node for all nodes representing geometric objects
class Object_node : public Base_node
{

public:

    typedef graphene::geometry::Geometry_representation Geometry_representation;
    typedef graphene::gl::GL_state GL_state;


    Object_node(const std::string& name="Object_node");
    virtual ~Object_node();

    const std::vector<std::string>& get_draw_modes_menu() const;
    void clear_draw_modes();
    void add_draw_mode(const std::string& _s);
    void remove_draw_mode(const std::string& _s);
    void set_draw_mode(int _i);
    void set_draw_mode(const std::string& s);
    std::string& get_draw_mode();
    int get_draw_mode_idx();


    const std::vector<std::string>& get_draw_more_settings_menu_all() const;
    void clear_draw_more_settings();
    void add_draw_more_settings(const std::string& _s);
    void remove_draw_more_settings(const std::string& _s);
    void toggle_draw_more_settings(int _i);
    std::vector<std::string> get_draw_more_settings_active();


    const std::vector<std::string>& get_selection_modes_menu() const;
    void clear_selection_modes();
    void add_selection_mode(const std::string& _s);
    void remove_selection_mode(const std::string& _s);
    void set_selection_mode(int _i);
    void set_selection_mode(const std::string& s);
    std::string& get_selection_mode();

    const std::vector<std::string>& get_manipulation_modes_menu() const;
    void clear_manipulation_modes();
    void add_manipulation_mode(const std::string& _s);
    void remove_manipulation_mode(const std::string& _s);
    void set_manipulation_mode(int _i);
    void set_manipulation_mode(const std::string& s);
    std::string& get_manipulation_mode();

    virtual void clear_selections() {}
    virtual void invert_selections() {}
    virtual void select_all() {}
    virtual void select_isolated() {}
    virtual void delete_selected() {}
    virtual void grow_selection() {}
    virtual void get_selection(std::vector<size_t>& indices) {}
    virtual void set_selection(const std::vector<size_t>& indices) {}
    virtual void clear_selection(const std::vector<size_t>& indices) {}
    virtual void add_selection(const std::vector<size_t>& indices) {}
    virtual std::vector<size_t> select_lasso(std::vector<Vec2f> lasso,
                                             GL_state* gl_state,
                                             bool visible_only);
    virtual void apply_transform(const Mat4f& transform) {}

    virtual float select_target(const Vec3f &point);
    virtual void select_point(Point p) {}
    virtual void select_landmark(const Point& p) {}
    virtual void undo_last_landmark() {}
    virtual void move_landmark(const int current_landmark_idx, const Point& p) {}
    virtual void clear_landmarks() {}
    virtual int  get_num_landmarks() { return 0; }
    virtual void highlight_landmark(int ) {}



protected:

    std::vector<std::string> draw_menu_;
    int                      draw_mode_;

    std::vector<std::string> draw_more_settings_menu_all_;
    std::vector<int>         draw_more_settings_active_;

    std::vector<std::string> selection_menu_;
    int                      selection_mode_;

    std::vector<std::string> manipulation_menu_;
    int                      manipulation_mode_;

    Geometry_representation* object_;


};


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_OBJECT_NODE_H
//=============================================================================
