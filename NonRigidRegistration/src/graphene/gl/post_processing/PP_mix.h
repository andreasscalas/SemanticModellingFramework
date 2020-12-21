
#ifndef GRAPHENE_PP_MIX_H
#define GRAPHENE_PP_MIX_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_mix : public Post_process
{
public:
    PP_mix();

    ~PP_mix();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif
