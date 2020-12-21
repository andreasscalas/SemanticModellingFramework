
#ifndef GRAPHENE_PP_SHADOW_MAPPING_H
#define GRAPHENE_PP_SHADOW_MAPPING_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/Framebuffer.h>

namespace graphene
{
namespace gl
{

class PP_shadow_mapping : public Post_process
{
    std::vector<int> texture_units_;

    std::vector<Mat4f> sm_matrices_;

public:
    float darkness_;


    PP_shadow_mapping();

    ~PP_shadow_mapping();

    void apply(GL_state *gls);

};

} //namespace gl
} //namespace graphene

#endif
