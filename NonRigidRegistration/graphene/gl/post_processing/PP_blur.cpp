
#include <graphene/gl/post_processing/PP_blur.h>



namespace graphene
{
namespace gl
{

PP_blur::PP_blur() :
    Post_process("Blur")
{

}

PP_blur::~PP_blur()
{

}

void PP_blur::apply(GL_state *gls)
{

    Shader* s = gls->set_active_shader(PP_BLUR_SHADER);

    Vec4i viewport;
    glGetIntegerv(GL_VIEWPORT, viewport.data());

    const Vec2f inv_res(1.0f/ viewport[2], 1.0f / viewport[3]);

    s->set_uniform("inv_resolution", inv_res);

    Framebuffer* fb;

    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 0);
    gls->post_processing_.get_linear_z_fb()->bind_depth_texture(s->get_shaderid(), "depth_texture", 1);
    s->set_uniform("direction", 0);

    fb = gls->post_processing_.get_write_fb();
    fb->bind();
    glClear(GL_COLOR_BUFFER_BIT);
    gls->post_processing_.draw_quad();

    gls->post_processing_.swap_buffers();

    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 0);
    s->set_uniform("direction", 1);

    fb = gls->post_processing_.get_write_fb();
    fb->bind();
    glClear(GL_COLOR_BUFFER_BIT);
    gls->post_processing_.draw_quad();

    gls->post_processing_.swap_buffers();
}

} //namespace gl
} //namespace graphene
