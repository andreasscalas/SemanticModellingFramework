 
#include <graphene/gl/post_processing/PP_shadow_mapping.h>



namespace graphene
{
namespace gl
{

PP_shadow_mapping::PP_shadow_mapping() :
    Post_process("Shadow Mapping"),
    darkness_(1.0f)
{
    texture_units_.reserve(4);
    sm_matrices_.reserve(4);

    //set number of samples per shadow map ( (num_samples * 2 + 1)^2 )
    // 1.0f would lead to (1 * 2 + 1)^2 = 9 samples
    //num_samples_ = 2.0f;


    //set shadow strength (higher value results in darker shadows)
    //1.0f means, that a point, that is in shadow from all light sources, has color 0.0
    //strength_ = 0.9f;

    needed_pps_.push_back(post_processing_hierarchy::LINEARIZE_Z);
    needed_pps_.push_back(post_processing_hierarchy::SHADOW_MAPPING);
    needed_pps_.push_back(post_processing_hierarchy::BLUR);
    needed_pps_.push_back(post_processing_hierarchy::MIX);

}

PP_shadow_mapping::~PP_shadow_mapping()
{

}

void PP_shadow_mapping::apply(GL_state *gls)
{

    if (gls->get_shadow_maps().empty())
        return;

    Shader* s;
    Framebuffer* fb;
    Shadow_map* sm;

    s = gls->set_active_shader(PP_SHADOWMAPPING_SHADER);

    //upload inverse camera viewprojection matrix
    s->set_uniform("viewproj_inv", inverse(gls->proj_ * gls->view_));

    s->set_uniform("strength", darkness_);

    //bind camera depth texture
    fb = gls->post_processing_.get_source_fb();
    fb->bind_depth_texture(s->get_shaderid(), "depth_texture", 0);

    //bind previous effect texture
    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 1);

    if (gls->get_shadow_maps().size() > 0)
    {
        sm = gls->get_shadow_maps()[0];
        s->bind_texture("shadowmap0", sm->get_depth_texture_id(), GL_TEXTURE_2D, 2);
        s->set_uniform("viewproj0", sm->viewproj_);
    }

    if (gls->get_shadow_maps().size() > 1)
    {
        sm = gls->get_shadow_maps()[1];
        s->bind_texture("shadowmap1", sm->get_depth_texture_id(), GL_TEXTURE_2D, 3);
        s->set_uniform("viewproj1", sm->viewproj_);
    }


    fb = gls->post_processing_.get_write_fb();
    fb->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    gls->post_processing_.draw_quad();

    gls->post_processing_.swap_buffers();

}



} //namespace gl
} //namespace graphene
