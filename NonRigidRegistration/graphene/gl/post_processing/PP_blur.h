

#ifndef GRAPHENE_PP_BLUR_H
#define GRAPHENE_PP_BLUR_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_blur : public Post_process
{
public:
    PP_blur();

    ~PP_blur();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif

