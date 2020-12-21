
#include <graphene/gl/post_processing/PP_linearize_z.h>



namespace graphene
{
namespace gl
{

PP_linearize_z::PP_linearize_z() :
    Post_process("Linearize Z")
{

}

PP_linearize_z::~PP_linearize_z()
{

}

void PP_linearize_z::apply(GL_state *gls)
{
    glEnable(GL_DEPTH_TEST);

    Shader* s;
    Framebuffer* fb;

    s = gls->set_active_shader(PP_LINEARIZE_Z_SHADER);

    s->set_uniform("near", gls->near_);
    s->set_uniform("far", gls->far_);

    fb = gls->post_processing_.get_source_fb();
    fb->bind_depth_texture(s->get_shaderid(), "depth_texture", 0);

    fb = gls->post_processing_.get_linear_z_fb();
    fb->bind();
    glClear(GL_DEPTH_BUFFER_BIT);
    gls->post_processing_.draw_quad();

    glDisable(GL_DEPTH_TEST);
}

} //namespace gl
} //namespace graphene
