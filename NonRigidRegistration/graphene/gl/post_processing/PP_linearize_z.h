

#ifndef GRAPHENE_PP_LINEARIZE_Z_H
#define GRAPHENE_PP_LINEARIZE_Z_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_linearize_z : public Post_process
{
public:
    PP_linearize_z();

    ~PP_linearize_z();

    void apply(GL_state *gls);
};

} //namespace gl
} //namespace graphene

#endif

