
#include <graphene/gl/post_processing/PP_subsurface_scattering.h>



namespace graphene
{
namespace gl
{

PP_subsurface_scattering::PP_subsurface_scattering() :
    Post_process("Subsurface Scattering"),
    level_(2.0f)
{

    needed_pps_.push_back(post_processing_hierarchy::LINEARIZE_Z);
    needed_pps_.push_back(post_processing_hierarchy::MIX);
    needed_pps_.push_back(post_processing_hierarchy::SUBSURFACE_SCATTERING);

    //weights for skin
    weights_.push_back(Vec3f(0.2405f, 0.4474f, 0.6157f));
    weights_.push_back(Vec3f(0.1158f, 0.3661f, 0.3439f));
    weights_.push_back(Vec3f(0.1836f, 0.1864f, 0.0f   ));
    weights_.push_back(Vec3f(0.4600f, 0.0f   , 0.0402f));

    standard_deviations_.push_back( (float) sqrt(0.0064));
    standard_deviations_.push_back( (float) sqrt(0.0516-0.0064));
    standard_deviations_.push_back( (float) sqrt(0.2719-0.0516));
    standard_deviations_.push_back( (float) sqrt(2.0062-0.2719));

    gauss_w_.resize(6);
    gauss_w_[0] = 0.006f;
    gauss_w_[1] = 0.061f;
    gauss_w_[2] = 0.242f;
    gauss_w_[3] = 0.242f;
    gauss_w_[4] = 0.061f;
    gauss_w_[5] = 0.006f;


    gauss_o_.resize(6);
    gauss_o_[0] = -1.0f;
    gauss_o_[1] = -0.6667f;
    gauss_o_[2] = -0.3333;
    gauss_o_[3] =  0.3333;
    gauss_o_[4] =  0.6667f;
    gauss_o_[5] =  1.0f;


/*
    //weights and standard devs for marble
    weights_.push_back(Vec3f(0.0544f, 0.1245f, 0.2177f));
    weights_.push_back(Vec3f(0.2436f, 0.2435f, 0.1890f));
    weights_.push_back(Vec3f(0.3105f, 0.3158f, 0.3742f));
    weights_.push_back(Vec3f(0.3913f, 0.3161f, 0.2189f));

    standard_deviations_.push_back( (float) sqrt(0.0362));
    standard_deviations_.push_back( (float) sqrt(0.1144-0.0362));
    standard_deviations_.push_back( (float) sqrt(0.1555-0.1144));
    standard_deviations_.push_back( (float) sqrt(3.4833-0.1555));
*/
/*
    //weights for skin
    weights_.push_back(Vec3f(0.2405f, 0.4474f, 0.6157f));
    weights_.push_back(Vec3f(0.1158f, 0.3661f, 0.3439f));
    weights_.push_back(Vec3f(0.1836f, 0.1864f, 0.0f   ));
    weights_.push_back(Vec3f(0.4600f, 0.0f   , 0.0402f));

    standard_deviations_.push_back( (float) sqrt(0.0064));
    standard_deviations_.push_back( (float) sqrt(0.0516-0.0064));
    standard_deviations_.push_back( (float) sqrt(0.2719-0.0516));
    standard_deviations_.push_back( (float) sqrt(2.0062-0.2719));
*/

    /*
    //weights and standard deviations from GPU gems
    weights_.push_back(Vec3f(0.233f, 0.455f, 0.649f));
    weights_.push_back(Vec3f(0.1f,   0.336f, 0.344f));
    weights_.push_back(Vec3f(0.118f, 0.198f, 0.0f  ));
    weights_.push_back(Vec3f(0.113f, 0.007f, 0.007f));
    weights_.push_back(Vec3f(0.358f, 0.004f, 0.0   ));
    weights_.push_back(Vec3f(0.078f, 0.0f  , 0.0   ));

    standard_deviations_.push_back( (float) sqrt(0.0064));
    standard_deviations_.push_back( (float) sqrt(0.0484-0.0064));
    standard_deviations_.push_back( (float) sqrt(0.187-0.0484));
    standard_deviations_.push_back( (float) sqrt(0.567-0.187));
    standard_deviations_.push_back( (float) sqrt(1.99-0.567));
    standard_deviations_.push_back( (float) sqrt(7.41-1.99));
    */
}

PP_subsurface_scattering::~PP_subsurface_scattering()
{

}

void PP_subsurface_scattering::apply(GL_state *gls)
{
    Vec4i viewport;
    Framebuffer* fb;
    Shader* s;


    glEnable(GL_BLEND);
    glBlendEquationi(fragdata_locations::COLOR_ALT, GL_FUNC_ADD);
    glBlendFunci(fragdata_locations::COLOR_ALT, GL_ONE, GL_ONE);

    fb = gls->post_processing_.get_read_fb();
    if (! fb->has_color_attachment(fragdata_locations::COLOR_ALT))
    {
        fb->set_color_attachment(fragdata_locations::COLOR_ALT, GL_RGBA16);
    }
    fb->bind();
    glClearBufferfv(GL_COLOR, fragdata_locations::COLOR_ALT, Vec4f(0.0f,0.0f,0.0f,1.0f).data());


    glGetIntegerv(GL_VIEWPORT, viewport.data());
    const Vec2f resolution_inv(1.0f/viewport[2], 1.0f/viewport[3]);


    s = gls->set_active_shader(PP_SUBSURFACE_SCATTERING_SHADER);

    s->set_uniform("inv_resolution", resolution_inv);
    s->set_uniform("ssslevel", level_);

    s->set_uniform("gauss_w", gauss_w_);
    s->set_uniform("gauss_o", gauss_o_);

    fb = gls->post_processing_.get_linear_z_fb();
    fb->bind_depth_texture(s->get_shaderid(), "depth_texture", 0);

    for (unsigned int i=0; i < weights_.size(); ++i)
    {
        s = gls->set_active_shader(PP_SUBSURFACE_SCATTERING_SHADER);

        s->set_uniform("standard_dev", standard_deviations_[i]);
        s->set_uniform("rgb_weights", weights_[i]);


        fb = gls->post_processing_.get_read_fb();
        fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 1);

        s->set_uniform("direction", 0);

        fb = gls->post_processing_.get_write_fb();
        fb->bind();
        glClearBufferfv(GL_COLOR, fragdata_locations::COLOR, gls->clear_color_.data());
        gls->post_processing_.draw_quad();


        gls->post_processing_.swap_buffers();

        fb = gls->post_processing_.get_read_fb();
        fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 1);

        s->set_uniform("direction", 1);

        fb = gls->post_processing_.get_write_fb();
        fb->bind();
        glClearBufferfv(GL_COLOR, fragdata_locations::COLOR, gls->clear_color_.data());
        gls->post_processing_.draw_quad();

        gls->post_processing_.swap_buffers();

    }

    gls->post_processing_.get_read_fb()->swap_attachments(fragdata_locations::COLOR, fragdata_locations::COLOR_ALT);

    glDisable(GL_BLEND);
}

} //namespace gl
} //namespace graphene
