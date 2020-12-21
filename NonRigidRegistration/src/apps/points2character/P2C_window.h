#ifndef POINTS2CHARACTER_WINDOW_H
#define POINTS2CHARACTER_WINDOW_H

#include <sstream>

#include <pmp/Window.h>
#include <opencl/CL_state.h>
#include <graphene/gl/GL_state.h>
#include <graphene/scene_graph/Scene_graph.h>
#include <graphene/gl/Trackball.h>

using namespace graphene;

namespace p2c
{


class P2C_window : public pmp::Window
{

private:
    gl::GL_state gl_state_;

    scene_graph::Scene_graph scene_graph_;

    gl::Trackball trackball_;

    std::stringstream task_ss_;

    //mouse helper
    bool button_down_[7];
    int modifiers_;

    bool pause_;


public:
    P2C_window(const char* title, int width, int height);

    void update();

    bool select_n_landmarks(size_t n_landmarks, const std::string &name_of_node);

    bool print_message_and_pause(const std::string& msg);

    void reset_view();

    void set_error_and_run(const std::string& error_msg = "");
    void set_done_and_run();

    //getter/setter
    scene_graph::Scene_graph &scene_graph() {return scene_graph_;}
    gl::GL_state &gl_state() {return gl_state_;}


private:

    void display();

    void processImGUI();

    void resize(int width, int height);

    void keyboard(int key, int code, int action, int mods);

    void character(unsigned int c);

    void mouse(int button, int action, int mods);

    void motion(double x, double y);

    void scroll(double x, double y);

    bool pick(int x, int y, Vec3f& result);

    void fly_to(int x, int y);

    void select_object(int x, int y);

    void select_landmark(int x, int y);


};

}



#endif
