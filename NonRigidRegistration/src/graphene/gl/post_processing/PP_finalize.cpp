 
#include <graphene/gl/post_processing/PP_finalize.h>



namespace graphene
{
namespace gl
{

PP_finalize::PP_finalize() :
    Post_process("Finalize")
{

}

PP_finalize::~PP_finalize()
{

}

void PP_finalize::apply(GL_state *gls)
{
    Framebuffer* fb;
    Shader* s = gls->set_active_shader(PP_FINAL_SHADER);


    fb = gls->post_processing_.get_source_fb();
    fb->bind_depth_texture(s->get_shaderid(), "depth_texture", 1);

    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 2);

    if (gls->multisampling_enabled_)
        glEnable(GL_MULTISAMPLE);

    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER,0);
    glEnable(GL_FRAMEBUFFER_SRGB);
    gls->post_processing_.draw_quad();
    glDisable(GL_FRAMEBUFFER_SRGB);

}

} //namespace gl
} //namespace graphene
