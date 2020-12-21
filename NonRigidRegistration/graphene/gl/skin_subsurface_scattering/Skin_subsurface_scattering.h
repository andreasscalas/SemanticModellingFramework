//=============================================================================
#ifndef GRAPHENE_SKIN_SUBSURFACE_SCATTERING_H
#define GRAPHENE_SKIN_SUBSURFACE_SCATTERING_H
//=============================================================================


#include <graphene/gl/Texture.h>
#include <graphene/gl/Framebuffer.h>
#include <graphene/gl/GL_state.h>
#include <graphene/gl/post_processing/Quad.h>


//=============================================================================


namespace graphene {
namespace gl {


//=============================================================================


/// \addtogroup gl
/// @{


class Skin_subsurface_scattering
{
public:

    Skin_subsurface_scattering(graphene::gl::GL_state* gl,
                               GLuint vertex_array_object,
                               unsigned int n_vertices,
                               graphene::gl::Texture* texture,
                               graphene::gl::Texture* texture_nm,
                               graphene::gl::Texture* texture_spec);

    ~Skin_subsurface_scattering();

    void apply(const float mixRatio, // 1 means full pre-scattering; 0 means full post-scattering; 0.5 is a good tradeoff
               const float factor_blurring, // factor blurring
               const Vec4f& material,
               const Vec3f& front_color,
               const Vec3f& back_color,
               const bool with_normal_map,
               const bool with_specular_map);


private:

    void init();

    bool initialized_;

    graphene::gl::Framebuffer_singlesample* stretch_correction_fb_;

    graphene::gl::Framebuffer_singlesample* irradiance_fb_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_temp_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_2_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_3_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_4_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_5_;
    graphene::gl::Framebuffer_singlesample* blurring_fb_6_;
    graphene::gl::Quad                      quad_;

    // standard deviations
    std::vector<float> standard_deviations_;

    // scale stretch correction
    float scale_stretch_correction_;

    // gl state
    graphene::gl::GL_state* gl_;

    // VAO
    GLuint vertex_array_object_;

    // number of vertices
    unsigned int n_vertices_;

    // albedo map
    graphene::gl::Texture* texture_;

    // normal map
    graphene::gl::Texture* texture_nm_;

    // specular map
    graphene::gl::Texture* texture_spec_;

};


//=============================================================================
/// @}
//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SKIN_SUBSURFACE_SCATTERING_H
//=============================================================================
