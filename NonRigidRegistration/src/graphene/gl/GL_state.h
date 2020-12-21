//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================

#ifndef GRAPHENE_GLSTATE_GL_H
#define GRAPHENE_GLSTATE_GL_H

//== INCLUDES =================================================================

#include "Shader.h"
#include <graphene/gl/Texture.h>
#include <graphene/gl/post_processing/Post_processing.h>
#include <graphene/gl/Shadow_map.h>
#include <graphene/geometry/Matrix3x3.h>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Bounding_box.h>
#include <vector>
#include <map>


//== NAMESPACE ================================================================

namespace graphene {
namespace gl {

//=============================================================================


enum Shader_type {
    PHONG_SHADER = 0,
    NORMALSPEC_PHONG_SHADER,
    SKINNING_PHONG_SHADER,
    SKINNING_NORMALSPEC_PHONG_SHADER,
    SPHERE_SHADER,
    CYLINDER_SHADER,
    POINT_SHADER,
    IRRADIANCE_SHADER,
    FINAL_SKIN_SHADER,

    //define new shader with calculation of a lightmodel above this entry and shaders without beneath this entry
    //setup_lights() method shows why
    SHADER_WITH_LIGHT_MODEL_MAX,

    MESH_SHADOWMAP_SHADER,
    SKINNING_SHADOWMAP_SHADER,
    SPHERE_SHADOWMAP_SHADER,
    CYLINDER_SHADOWMAP_SHADER,

    TEXTURE_SHADER,
    STRETCH_CORRECTION_SHADER,
    BLURRING_SHADER,

    PP_AMBIENTOCCLUSION_SHADER,
    PP_BLUR_SHADER,
    PP_LINEARIZE_Z_SHADER,
    PP_SHADOWMAPPING_SHADER,
    PP_SUBSURFACE_SCATTERING_SHADER,
    PP_MIX_SHADER,
    PP_FINAL_SHADER,

    SLIDE_SHADER,

    SHADER_MAX
};


//=============================================================================


struct Camera
{
    Vec3f eye_;
    Vec3f center_;
    Vec3f up_;

    Camera() :
        eye_(0.0f), center_(0.0f), up_(0.0f,1.0f,0.0f)
    {}
};


//=============================================================================


struct Light
{
    Light() {}
    Light(const Vec3f& _position,
          const Vec3f& _color) :
        position_(_position), color_(_color)
    {}

    Vec3f position_;
    Vec3f color_;
};


//=============================================================================


class GL_state
{
public:

    GL_state();
    ~GL_state();

    bool init(const char* argv);

    //makes 'shader_type' the current shader
    Shader* set_active_shader(Shader_type shader_type);
    //returns a pointer to the currently active shader
    Shader* get_active_shader();
    //get shader type of active shader
    Shader_type get_active_shader_type();

    //setup camera and corresponding matrices so that the whole scene is visible
    void view_all();

    //translate the camera position
    void translate_camera(const Vec3f &t);

    void fly_to(const Vec3f& p);

    //update the projection matrix based on width and height
    void update_projection();

    //update mvp and normal matrix based on current model, view and projection matrices
    void update_matrices();

    //bind texture to currently active shader
    void bind_texture(const char* sampler_name,
                      GLuint texture_id,
                      GLint texture_type,
                      GLint texture_unit);

    //set up static lights for all shaders
    void setup_lights(const std::vector<Light>& _lights);

    void setup_shadowmaps(const Vec3f& scene_center, float radius);

    const std::vector<Shadow_map*>& get_shadow_maps();

    /// adds a texture to the heap
    void add_texture_to_heap(Texture* texture);
    ///returns texture from texture heap with name tex_name. If not present, NULL is returned.
    Texture* get_texture_from_heap(const std::string& tex_name);
    /// clears all textures and deletes them. Do not use them anywhere afterwards!
    void clear_texture_heap();

    //getter/setter
    void set_width(float width){width_ = width;}
    void set_height(float height){height_ = height;}
    void set_near(float n) {near_ = n;}
    void set_far(float f) {far_ = f;}
    void set_bbox(const geometry::Bounding_box& bbox) {bbox_ = bbox;}

    const Camera& get_camera(){ return camera_; }

    const Mat4f& get_model(){return model_;}
    const Mat4f& get_view(){return view_;}
    const Mat4f& get_proj(){return proj_;}
    const Mat4f& get_modelview(){return modelview_;}
    const Mat4f& get_modelviewproj(){return modelviewproj_;}
    const Mat3f& get_normal(){return normal_;}
private:

    bool load_shaders(const char* argv);
    Shader* load_shader(const char* vs_file,
                        const char* fs_file,
                        const char* gs_file=NULL);


public:

    //public matrices
    Mat4f model_;
    Mat4f view_;
    Mat4f proj_;

    //these matrices are only up to date after a call from update_matrices()!!!
    Mat4f modelviewproj_;
    Mat4f modelview_;
    Mat3f normal_;

    // projection parameters
    float fovy_, width_, height_, near_, far_;

    //clear color
    Vec4f clear_color_;

    //multisample enabled?
    bool multisampling_enabled_;

    // light sources
    std::vector<Light> lights_;

    Post_processing post_processing_;

    std::map<std::string, Texture*> texture_heap_;

    bool use_shadowmaps_;

private:
    geometry::Bounding_box bbox_;

    Camera camera_;

    //shaders
    std::vector<Shader*> shaders_;

    //shadow maps
    std::vector<Shadow_map*> shadow_maps_;

    //saves the active shader type;
    Shader_type shader_type_;
};


//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_GLSTATE_GL_H
//=============================================================================
