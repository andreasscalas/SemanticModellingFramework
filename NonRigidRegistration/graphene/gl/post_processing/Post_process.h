
#ifndef GRAPHENE_POST_PROCESS_H
#define GRAPHENE_POST_PROCESS_H

#include <graphene/gl/GL_state.h>

namespace graphene
{
namespace gl
{

namespace post_processing_hierarchy
{

//the order is important here!!!
//for example: linearize z must be done before ambientocclusion or blur
//OR mixing of colors before subsurface scattering
enum Post_processing_hierarchy
{
    LINEARIZE_Z = 0,
    AMBIENTOCCLUSION,
    SHADOW_MAPPING,
    BLUR,
    MIX,
    SUBSURFACE_SCATTERING,
    FINALIZE,
    MAX
};

}

struct Post_process
{
    std::string name_;

    //only the "real" post processes fill this vector
    std::vector<post_processing_hierarchy::Post_processing_hierarchy> needed_pps_;

    bool active_;

    Post_process(const std::string& name = "No Name") :
        name_(name), active_(false)
    { }

    virtual ~Post_process(){}

    virtual void apply(GL_state* gls) = 0;
};

} //namespace gl
} //namespace graphene

#endif
