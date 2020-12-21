
#ifndef GRAPHENE_PP_FINALIZE_H
#define GRAPHENE_PP_FINALIZE_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_finalize : public Post_process
{
public:
    PP_finalize();

    ~PP_finalize();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif
