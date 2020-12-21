

#ifndef GRAPHENE_PP_SUBSURFACE_SCATTERING_H
#define GRAPHENE_PP_SUBSURFACE_SCATTERING_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_subsurface_scattering : public Post_process
{
private:
    std::vector<Vec3f> weights_;

    std::vector<float> standard_deviations_;

    std::vector<float> gauss_w_;
    std::vector<float> gauss_o_;

public:
    float level_;

    PP_subsurface_scattering();

    ~PP_subsurface_scattering();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif

