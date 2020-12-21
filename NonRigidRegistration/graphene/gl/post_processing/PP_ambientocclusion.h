
#ifndef GRAPHENE_PP_AMBIENTOCCLUSION_H
#define GRAPHENE_PP_AMBIENTOCCLUSION_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_ambientocclusion : public Post_process
{
    GLuint random_texture_id_;

public:
    float contrast_;

    PP_ambientocclusion();

    ~PP_ambientocclusion();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif
