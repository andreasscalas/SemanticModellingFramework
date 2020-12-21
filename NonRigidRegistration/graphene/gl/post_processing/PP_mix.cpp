 
#include <graphene/gl/post_processing/PP_mix.h>



namespace graphene
{
namespace gl
{

PP_mix::PP_mix() :
    Post_process("Color Mix")
{

}

PP_mix::~PP_mix()
{

}

void PP_mix::apply(GL_state *gls)
{
    Framebuffer* fb;
    Shader* s = gls->set_active_shader(PP_MIX_SHADER);

    fb = gls->post_processing_.get_fb(2);
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "color_texture", 0);

    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 1);

    fb = gls->post_processing_.get_write_fb();
    fb->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    gls->post_processing_.draw_quad();

    gls->post_processing_.swap_buffers();
}

} //namespace gl
} //namespace graphene
