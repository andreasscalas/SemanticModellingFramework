 
#include <graphene/gl/post_processing/PP_ambientocclusion.h>
#include <cstdlib>

namespace graphene
{
namespace gl
{

PP_ambientocclusion::PP_ambientocclusion() :
    Post_process("Ambient Occlusion"),
    random_texture_id_(0),
    contrast_(1.0f)
{

    needed_pps_.push_back(post_processing_hierarchy::LINEARIZE_Z);
    needed_pps_.push_back(post_processing_hierarchy::AMBIENTOCCLUSION);
    needed_pps_.push_back(post_processing_hierarchy::BLUR);
    needed_pps_.push_back(post_processing_hierarchy::MIX);

    int size = 64*64*3;
    float* data = new float[size];
    srand(98274);
    double angle;
    double random;

    for (int i=0; i<size; i+=3)
    {
        random = (rand() % 1000000) * 0.000001;
        angle = 2.0 * M_PI * random/16.0;
        data[i  ] = (float)cos(angle);
        data[i+1] = (float)sin(angle);
        data[i+2] = (rand() % 1000000) * 0.000001;
    }

    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &random_texture_id_);
    glBindTexture(GL_TEXTURE_2D, random_texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64,
                 0, GL_RGB, GL_FLOAT, data);

    delete[] data;
}

PP_ambientocclusion::~PP_ambientocclusion()
{
    if (random_texture_id_)
        glDeleteTextures(1, &random_texture_id_);

}

void PP_ambientocclusion::apply(GL_state *gls)
{
    Shader* s;
    Framebuffer* fb;
    Vec4i viewport;

    s = gls->set_active_shader(PP_AMBIENTOCCLUSION_SHADER);

    glGetIntegerv(GL_VIEWPORT, viewport.data());

    const Vec2f resolution(viewport[2], viewport[3]);
    const Vec2f inv_reso(1.0f/resolution[0], 1.0f/resolution[1]);
    s->set_uniform("resolution", resolution);
    s->set_uniform("inv_resolution", inv_reso);
    s->set_uniform("Contrast", contrast_);


    fb = gls->post_processing_.get_read_fb();
    fb->bind_color_texture(fragdata_locations::COLOR, s->get_shaderid(), "effect_texture", 0);

    fb = gls->post_processing_.get_linear_z_fb();
    fb->bind_depth_texture(s->get_shaderid(), "depth_texture", 1);

    s->bind_texture("randomtex", random_texture_id_, GL_TEXTURE_2D, 2);


    fb = gls->post_processing_.get_write_fb();
    fb->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    gls->post_processing_.draw_quad();

    gls->post_processing_.swap_buffers();

}

} //namespace gl
} //namespace graphene
