//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


//#include <graphene/gl/gl_includes.h>
#include <graphene/gl/skin_subsurface_scattering/Skin_subsurface_scattering.h>


//=============================================================================


namespace graphene {
namespace gl {


//=============================================================================


Skin_subsurface_scattering::
Skin_subsurface_scattering(graphene::gl::GL_state* gl,
                           GLuint vertex_array_object,
                           unsigned int n_vertices,
                           graphene::gl::Texture* texture,
                           graphene::gl::Texture* texture_nm,
                           graphene::gl::Texture* texture_spec) :
    initialized_(false),
    stretch_correction_fb_(0),
    irradiance_fb_(0),
    blurring_fb_temp_(0),
    blurring_fb_2_(0),
    blurring_fb_3_(0),
    blurring_fb_4_(0),
    blurring_fb_5_(0),
    blurring_fb_6_(0),
    scale_stretch_correction_(1.0f),
    gl_(gl),
    vertex_array_object_(vertex_array_object),
    n_vertices_(n_vertices),
    texture_(texture),
    texture_nm_(texture_nm),
    texture_spec_(texture_spec)
{
    standard_deviations_.push_back( std::sqrt(0.0064) );
    standard_deviations_.push_back( std::sqrt(0.0484 - 0.0064) );
    standard_deviations_.push_back( std::sqrt(0.187  - 0.0484) );
    standard_deviations_.push_back( std::sqrt(0.567  - 0.187) );
    standard_deviations_.push_back( std::sqrt(1.99   - 0.567) );
    standard_deviations_.push_back( std::sqrt(7.41   - 1.99) );

    init();
}


//-----------------------------------------------------------------------------


Skin_subsurface_scattering::
~Skin_subsurface_scattering()
{
    if (stretch_correction_fb_)     delete stretch_correction_fb_;
    if (irradiance_fb_)             delete irradiance_fb_;
    if (blurring_fb_temp_)          delete blurring_fb_temp_;
    if (blurring_fb_2_)             delete blurring_fb_2_;
    if (blurring_fb_3_)             delete blurring_fb_3_;
    if (blurring_fb_4_)             delete blurring_fb_4_;
    if (blurring_fb_5_)             delete blurring_fb_5_;
    if (blurring_fb_6_)             delete blurring_fb_6_;
}


//-----------------------------------------------------------------------------


void
Skin_subsurface_scattering::
init()
{
    if (initialized_)
    {
        return;
    }

    // init quad to render to off-screen texture
    quad_.init();

    // initialize framebuffers
    if(!stretch_correction_fb_)
    {
        stretch_correction_fb_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RG32F);
        graphene::gl::Attachment* a = stretch_correction_fb_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!irradiance_fb_)
    {
        irradiance_fb_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = irradiance_fb_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_temp_)
    {
        blurring_fb_temp_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_temp_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_2_)
    {
        blurring_fb_2_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_2_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_3_)
    {
        blurring_fb_3_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_3_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_4_)
    {
        blurring_fb_4_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_4_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_5_)
    {
        blurring_fb_5_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_5_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }
    if(!blurring_fb_6_)
    {
        blurring_fb_6_ = new gl::Framebuffer_singlesample(texture_->width_, texture_->height_, false, GL_RGBA8);
        graphene::gl::Attachment* a = blurring_fb_6_->get_color_attachment();
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // TODO TEST
//        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // TODO TEST
    }

    // backup current viewport dimensions
    Vec4i viewport;
    glGetIntegerv(GL_VIEWPORT, viewport.data());

    // change viewport for the off-screen texture
    glViewport(0, 0, texture_->width_, texture_->height_);

    // precompute stretch correction map
    glDisable(GL_DEPTH_TEST);

    // activate stretch correction shader
    gl::Shader* stretch_correction_shader = gl_->set_active_shader(gl::STRETCH_CORRECTION_SHADER);

    stretch_correction_shader->set_uniform("model_matrix", gl_->model_);
    stretch_correction_shader->set_uniform("scale", scale_stretch_correction_);

    // render stretch correction into off-screen texture
    stretch_correction_fb_->bind();
    glClear(GL_COLOR_BUFFER_BIT);
    glBindVertexArray(vertex_array_object_);
    glDrawArrays(GL_TRIANGLES, 0, n_vertices_);

    // revert viewport for main window
    glViewport(0, 0, viewport[2],viewport[3]);

    glEnable(GL_DEPTH_TEST);

    initialized_ = true;
}


//-----------------------------------------------------------------------------


void
Skin_subsurface_scattering::
apply(const float mixRatio, const float factor_blurring,
      const Vec4f& material,
      const Vec3f& front_color,
      const Vec3f& back_color,
      const bool with_normal_map,
      const bool with_specular_map)
{
    // backup current viewport dimensions
    Vec4i viewport;
    glGetIntegerv(GL_VIEWPORT, viewport.data());

    // change viewport for the off-screen texture
    glViewport(0, 0, texture_->width_, texture_->height_);

    glDisable(GL_DEPTH_TEST);

    // compute irradiance map

    // activate irradiance shader
    gl::Shader* irradiance_shader = gl_->set_active_shader(gl::IRRADIANCE_SHADER);
    irradiance_shader->set_uniform("normal_matrix", gl_->normal_);
    irradiance_shader->set_uniform("modelview_matrix", gl_->modelview_);
    irradiance_shader->set_uniform("model_matrix", gl_->model_);
    irradiance_shader->set_uniform("mixRatio", mixRatio);
    irradiance_shader->bind_texture("albedomap", texture_->id_, GL_TEXTURE_2D, 1);
    irradiance_shader->bind_texture("texture2D_nm", texture_nm_->id_, GL_TEXTURE_2D, 2);
    irradiance_shader->set_uniform("use_texture2D", true);



    if (with_normal_map)
    {
        irradiance_shader->set_uniform("use_normalmap", true);
    }
    else
    {
        irradiance_shader->set_uniform("use_normalmap", false);
    }



// render shadow maps   (TODO ALREADY DONE in paintGL(...)) - TODO: BOOL !!!

    // shadow map stuff

    graphene::gl::Shadow_map* sm;

    if (gl_->use_shadowmaps_)
    {
        irradiance_shader->set_uniform("use_shadowmaps", true);

        irradiance_shader->set_uniform("strength", 0.75f);

        if (gl_->get_shadow_maps().size() > 0)
        {
            sm = gl_->get_shadow_maps()[0];
            irradiance_shader->bind_texture("shadowmap0", sm->get_depth_texture_id(), GL_TEXTURE_2D, 3);
            irradiance_shader->set_uniform("viewproj0", sm->viewproj_);
        }

        if (gl_->get_shadow_maps().size() > 1)
        {
            sm = gl_->get_shadow_maps()[1];
            irradiance_shader->bind_texture("shadowmap1", sm->get_depth_texture_id(), GL_TEXTURE_2D, 4);
            irradiance_shader->set_uniform("viewproj1", sm->viewproj_);
        }
    }
    else
    {
        irradiance_shader->set_uniform("use_shadowmaps", false);
    }


    // render irradiance into off-screen texture
    irradiance_fb_->bind(); // bind to texture's FBO
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, n_vertices_);





    // FILL BLURRED IRRADIANCE MAP NO. 2

        
    // blurring in U direction

    // bind blurring framebuffer (temp)
    blurring_fb_temp_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // activate blurring shader
    gl::Shader* blurring_shader = gl_->set_active_shader(gl::BLURRING_SHADER);
    // configure blurring shader
    const Vec2f inv_reso(1.0f/viewport[2], 1.0f/viewport[3]);
    blurring_shader->set_uniform("inv_resolution", inv_reso);
    blurring_shader->set_uniform("factor", factor_blurring);
    blurring_shader->set_uniform("scale", scale_stretch_correction_);
    blurring_shader->set_uniform("GaussWidth", standard_deviations_[1]);
    blurring_shader->set_uniform("direction", 0);

    // use the texture that is associated with the FBO
    irradiance_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();

    // blurring in V direction

    // bind blurring framebuffer 2
    blurring_fb_2_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // configure blurring shader
    blurring_shader->set_uniform("direction", 1);

    // use the texture that is associated with the FBO
    blurring_fb_temp_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();





    // FILL BLURRED IRRADIANCE MAP NO. 3

    // blurring in U direction

    // bind blurring framebuffer (temp)
    blurring_fb_temp_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // activate blurring shader
    blurring_shader->set_uniform("GaussWidth", standard_deviations_[2]);
    blurring_shader->set_uniform("direction", 0);

    // use the texture that is associated with the FBO
    blurring_fb_2_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();

    // blurring in V direction

    // bind blurring framebuffer 2
    blurring_fb_3_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // configure blurring shader
    blurring_shader->set_uniform("direction", 1);

    // use the texture that is associated with the FBO
    blurring_fb_temp_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();








    // FILL BLURRED IRRADIANCE MAP NO. 4


    // blurring in U direction

    // bind blurring framebuffer (temp)
    blurring_fb_temp_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // activate blurring shader
    blurring_shader->set_uniform("GaussWidth", standard_deviations_[3]);
    blurring_shader->set_uniform("direction", 0);

    // use the texture that is associated with the FBO
    blurring_fb_3_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();

    // blurring in V direction

    // bind blurring framebuffer 2
    blurring_fb_4_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // configure blurring shader
    blurring_shader->set_uniform("direction", 1);

    // use the texture that is associated with the FBO
    blurring_fb_temp_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();







    // FILL BLURRED IRRADIANCE MAP NO. 5


    // blurring in U direction

    // bind blurring framebuffer (temp)
    blurring_fb_temp_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // activate blurring shader
    blurring_shader->set_uniform("GaussWidth", standard_deviations_[4]);
    blurring_shader->set_uniform("direction", 0);

    // use the texture that is associated with the FBO
    blurring_fb_4_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();

    // blurring in V direction

    // bind blurring framebuffer 2
    blurring_fb_5_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // configure blurring shader
    blurring_shader->set_uniform("direction", 1);

    // use the texture that is associated with the FBO
    blurring_fb_temp_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();



        





    // FILL BLURRED IRRADIANCE MAP NO. 6


    // blurring in U direction

    // bind blurring framebuffer (temp)
    blurring_fb_temp_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // activate blurring shader
    blurring_shader->set_uniform("GaussWidth", standard_deviations_[5]);
    blurring_shader->set_uniform("direction", 0);

    // use the texture that is associated with the FBO
    blurring_fb_5_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();

    // blurring in V direction

    // bind blurring framebuffer 2
    blurring_fb_6_->bind();
    glClear(GL_COLOR_BUFFER_BIT);

    // configure blurring shader
    blurring_shader->set_uniform("direction", 1);

    // use the texture that is associated with the FBO
    blurring_fb_temp_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "texture_to_blur", 2);
    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, blurring_shader->get_shaderid(), "stretch_map", 3);

    quad_.draw();





    // FINAL SKIN SHADER / final render pass

    // back to default FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // revert viewport for main window
    glViewport(0, 0, viewport[2],viewport[3]);

    glBindVertexArray(vertex_array_object_);

    // activate shader to display texture on screen
    gl::Shader* final_shader = gl_->set_active_shader(gl::FINAL_SKIN_SHADER);

    // set uniforms
    final_shader->set_uniform("use_lighting",    false);
    final_shader->set_uniform("use_vertexcolor", false);
    final_shader->set_uniform("use_texture2D",     false);
    final_shader->set_uniform("material", material);
    final_shader->set_uniform("front_color", front_color);
    final_shader->set_uniform("back_color", back_color);
    final_shader->set_uniform("modelview_projection_matrix", gl_->modelviewproj_);
    final_shader->set_uniform("modelview_matrix", gl_->modelview_);
    final_shader->set_uniform("model_matrix", gl_->model_);
    final_shader->set_uniform("normal_matrix", gl_->normal_);
// TODO WAR AN    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
// TODO WAR AN    glBindBuffer(GL_ARRAY_BUFFER, diffuse_texcoord_buffer_);
// TODO WAR AN    glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    final_shader->set_uniform("use_texture2D", true);
    final_shader->bind_texture("texture2D", texture_->id_, GL_TEXTURE_2D, 1);
    final_shader->bind_texture("texture2D_nm", texture_nm_->id_, GL_TEXTURE_2D, 2);
    final_shader->bind_texture("texture2D_spec", texture_spec_->id_, GL_TEXTURE_2D, 3);
    final_shader->set_uniform("front_color", Vec3f(1.0f));
    final_shader->set_uniform("use_lighting", true);

    // use the textures that are associated with the FBOs
    // blurred irradiance textures
    irradiance_fb_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad1Tex", 4);
    blurring_fb_2_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad2Tex", 5);
    blurring_fb_3_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad3Tex", 6);
    blurring_fb_4_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad4Tex", 7);
    blurring_fb_5_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad5Tex", 8);
    blurring_fb_6_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader->get_shaderid(), "irrad6Tex", 9);

    // mix ratio
    final_shader->set_uniform("mixRatio", mixRatio);

    // RGB Gaussian weights that define skin profiles
    final_shader->set_uniform("gauss1w", Vec3f(0.233f, 0.455f, 0.649f));
    final_shader->set_uniform("gauss2w", Vec3f(0.1f, 0.336f, 0.344f));
    final_shader->set_uniform("gauss3w", Vec3f(0.118f, 0.198f, 0.0f));
    final_shader->set_uniform("gauss4w", Vec3f(0.113f, 0.007f, 0.007f));
    final_shader->set_uniform("gauss5w", Vec3f(0.358f, 0.004f, 0.0f));
    final_shader->set_uniform("gauss6w", Vec3f(0.078f, 0.0f, 0.0f));



    if (with_normal_map)
    {
        final_shader->set_uniform("use_normalmap", true);
    }
    else
    {
        final_shader->set_uniform("use_normalmap", false);
    }


    if (with_specular_map)
    {
        final_shader->set_uniform("use_specularmap", true);
    }
    else
    {
        final_shader->set_uniform("use_specularmap", false);
    }


    if (gl_->use_shadowmaps_)
    {
        final_shader->set_uniform("use_shadowmaps", true);
        
        final_shader->set_uniform("strength", 0.75f);

        if (gl_->get_shadow_maps().size() > 0)
        {
            sm = gl_->get_shadow_maps()[0];
            final_shader->bind_texture("shadowmap0", sm->get_depth_texture_id(), GL_TEXTURE_2D, 10);
            final_shader->set_uniform("viewproj0", sm->viewproj_);
        }

        if (gl_->get_shadow_maps().size() > 1)
        {
            sm = gl_->get_shadow_maps()[1];
            final_shader->bind_texture("shadowmap1", sm->get_depth_texture_id(), GL_TEXTURE_2D, 11);
            final_shader->set_uniform("viewproj1", sm->viewproj_);
        }
    }
    else
    {
        final_shader->set_uniform("use_shadowmaps", false);
    }



    glDepthRange(0.002, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_FRAMEBUFFER_SRGB);
    glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
    glDisable(GL_FRAMEBUFFER_SRGB);


//    gl::Shader* final_shader2 = gl->set_active_shader(gl::TEXTURE_SHADER);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    // use the texture that is associated with the FBO
//    stretch_correction_fb_->bind_color_texture(gl::fragdata_locations::COLOR, final_shader2->get_shaderid(), "texture_of_interest", 2);

//    glEnable(GL_FRAMEBUFFER_SRGB);
//    quad_.draw();
//    glDisable(GL_FRAMEBUFFER_SRGB);
//    glEnable(GL_DEPTH_TEST);
}


//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
