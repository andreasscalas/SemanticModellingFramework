//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


//== INCLUDES =================================================================


#include "GL_state.h"

#include <opencl/CL_state.h>


//=============================================================================


namespace graphene {
namespace gl {


//=============================================================================


GL_state::GL_state() :
    model_(Mat4f::identity()),
    view_(Mat4f::identity()),
    proj_(Mat4f::identity()),
    modelviewproj_(Mat4f::identity()),
    modelview_(Mat4f::identity()),
    normal_(Mat3f::identity()),
    fovy_(45.0f),
    width_(80.0f),
    height_(80.0f),
    near_(0.01f),
    far_(10.0f),
    clear_color_(1.0f),
    use_shadowmaps_(false),
    shaders_(SHADER_MAX, NULL),
    shader_type_(PHONG_SHADER)
{
}


//-----------------------------------------------------------------------------


GL_state::~GL_state()
{
    unsigned int i;
    for (i=0; i < shaders_.size(); ++i)
    {
        if (shaders_[i]) delete shaders_[i];
    }

    //twaltema: clean up textures on delete
    clear_texture_heap();

}


//-----------------------------------------------------------------------------


bool GL_state::init(const char *argv)
{
    if (!load_shaders(argv))
    {
        return false;
    }

    int num_multisamples = 0;
    glGetIntegerv(GL_SAMPLES, &num_multisamples);
    if (num_multisamples > 0)
    {
        multisampling_enabled_ = true;
    }
    else
    {
        multisampling_enabled_ = false;
    }

    return true;
}


//-----------------------------------------------------------------------------


Shader* GL_state::set_active_shader(Shader_type shader_type)
{
    shader_type_ = shader_type;
    shaders_[shader_type_]->use();
    return get_active_shader();
}


//-----------------------------------------------------------------------------


Shader* GL_state::get_active_shader()
{
    return shaders_[shader_type_];
}


//-----------------------------------------------------------------------------


Shader_type GL_state::get_active_shader_type()
{
    return shader_type_;
}


//-----------------------------------------------------------------------------

void GL_state::view_all()
{
    model_ = Mat4f::identity();

    camera_.center_ = bbox_.center();
    camera_.eye_    = camera_.center_ + Vec3f(0.0f, 0.0f, bbox_.size());
    view_           = Mat4f::look_at(camera_.eye_, camera_.center_, camera_.up_);

    //near_           = 0.01 * bbox_.size();
    //far_            = 10.0 * bbox_.size();
    update_projection();
}

//-----------------------------------------------------------------------------

void GL_state::translate_camera(const Vec3f &t)
{
    camera_.center_ += t;
    camera_.eye_    += t;

    view_ = Mat4f::look_at(camera_.eye_, camera_.center_, camera_.up_);
}

//-----------------------------------------------------------------------------

void GL_state::fly_to(const Vec3f &p)
{
    const Vec3f t = 0.3f * affine_transform(modelview_, p);
    translate_camera(t);
}

//-----------------------------------------------------------------------------

void GL_state::update_projection()
{
    proj_ = Mat4f::perspective(fovy_, width_/height_, near_, far_);
}

//-----------------------------------------------------------------------------


void GL_state::update_matrices()
{
    modelview_     = view_*model_;

    const Vec3f center = bbox_.center();
    const float radius = bbox_.size()*0.5f;

    float z = -(modelview_[ 2]*center[0] + modelview_[ 6]*center[1] + modelview_[10]*center[2] + modelview_[14]);
    near_ = std::max(z-radius, 0.001f*radius);
    far_  = near_ + 2.0f*radius;

    update_projection();

    modelviewproj_ = proj_*modelview_;
    normal_        = inverse(transpose(Mat3f(modelview_)));
}


//-----------------------------------------------------------------------------


void GL_state::bind_texture(const char *sampler_name,
                            GLuint texture_id,
                            GLint texture_type,
                            GLint texture_unit)
{
    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(texture_type, texture_id);

    GLint id = glGetUniformLocation(Shader::get_active()->get_shaderid(), sampler_name);
    // if (id == -1)
    // {
    //     std::cerr << "Error: Invalid texture name (in GL_state)!" << std::endl;
    // }
    glUniform1i(id, texture_unit);

    glActiveTexture(GL_TEXTURE0);

}


//-----------------------------------------------------------------------------


void GL_state::setup_lights(const std::vector<Light>& _lights)
{
    // store light sources
    lights_ = _lights;


    // let's accept only 8 light sources
    if (lights_.size() > 8)
    {
        std::cerr << "GL_state: only 8 light sources possible\n";
        lights_.resize(8);
    }


    // setup data to be used in shaders
    std::vector<Vec3f> light_dirs;
    std::vector<Vec3f> light_colors;
    for (unsigned int i=0; i<lights_.size(); ++i)
    {
        light_dirs.push_back(normalize(-lights_[i].position_));
        light_colors.push_back(lights_[i].color_);
    }


    // upload light data to each shader
    Shader* shader;
    for (unsigned int i=0; i < SHADER_WITH_LIGHT_MODEL_MAX; ++i)
    {
        shader = shaders_[i];
        if (shader)
        {
            shader->use();
            shader->set_uniform("light_directions", light_dirs);
            shader->set_uniform("light_colors", light_colors);
            shader->set_uniform("num_active_lights", (int)light_dirs.size());
        }
    }
}


//-----------------------------------------------------------------------------

void GL_state::setup_shadowmaps(const Vec3f &scene_center, float radius)
{
    while (shadow_maps_.size() < 2 && shadow_maps_.size() < lights_.size())
    {
        shadow_maps_.push_back(new Shadow_map(2048, 2048));
    }



    unsigned int i;
    Shadow_map* sm;
    Light light;
    for (i=0; i < shadow_maps_.size(); ++i)
    {
        sm = shadow_maps_[i];
        light = lights_[i];
        sm->setup(scene_center, normalize(-light.position_), radius);
    }

}

//-----------------------------------------------------------------------------

const std::vector<Shadow_map*>& GL_state::get_shadow_maps()
{
    return shadow_maps_;
}

//-----------------------------------------------------------------------------

void GL_state::add_texture_to_heap(Texture *texture)
{
    texture_heap_[texture->name_] = texture;
}

//-----------------------------------------------------------------------------

Texture* GL_state::get_texture_from_heap(const std::string &tex_name)
{
    if (texture_heap_.count(tex_name) > 0)
    {
        return texture_heap_[tex_name];
    }
    else
    {
        return NULL;
    }
}

//-----------------------------------------------------------------------------

void GL_state::clear_texture_heap()
{
    std::map<std::string, Texture*>::iterator it;
    for (it = texture_heap_.begin(); it != texture_heap_.end(); ++it)
    {
        delete it->second;
    }
    texture_heap_.clear();
}

//-----------------------------------------------------------------------------


bool GL_state::load_shaders(const char *argv)
{
    // path to shader directory
    std::string shader_path(argv);
#ifdef _WIN32
    shader_path = shader_path.substr(0, shader_path.find_last_of('\\')+1);
#elif __APPLE__
    shader_path = shader_path.substr(0, shader_path.find_last_of('/')+1);
    //shader_path += "../"; // need when compiling with Xcode, not for gcc
#else
    shader_path = shader_path.substr(0, shader_path.find_last_of('/')+1);
#endif
    shader_path += "shaders/";


    // Phong shader
    shaders_[PHONG_SHADER] = load_shader((shader_path + "phong.vs").c_str(),
                                         (shader_path + "phong.fs").c_str());

    // Skinning Phong shader
    shaders_[SKINNING_PHONG_SHADER] = load_shader((shader_path + "skinning.vs").c_str(),
                                                  (shader_path + "phong.fs").c_str());


    // sphere shader
    shaders_[SPHERE_SHADER] = load_shader((shader_path + "sphere.vsh").c_str(),
                                          (shader_path + "sphere.fsh").c_str());

    // cylinder shader
    shaders_[CYLINDER_SHADER] = load_shader((shader_path + "cylinder.vsh").c_str(),
                                            (shader_path + "cylinder.fsh").c_str());

    // texture shader
    shaders_[TEXTURE_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                            (shader_path + "texture.fsh").c_str());

    // stretch correction shader
    shaders_[STRETCH_CORRECTION_SHADER] = load_shader((shader_path + "unwrap.vsh").c_str(),
                                                      (shader_path + "stretch_correction.fsh").c_str());

    // blurring shader
    shaders_[BLURRING_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                            (shader_path + "blur.fsh").c_str());

    // point shader
    shaders_[POINT_SHADER] = load_shader((shader_path + "point.vsh").c_str(),
                                            (shader_path + "point.fsh").c_str());

    //post processing ao shader
    shaders_[PP_AMBIENTOCCLUSION_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                                    (shader_path + "pp_ambientocclusion.fsh").c_str());

    //post processing blur shader
    shaders_[PP_BLUR_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                        (shader_path + "pp_blur.fsh").c_str());

    //post processing z linearizer
    shaders_[PP_LINEARIZE_Z_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                               (shader_path + "pp_linearize_z.fsh").c_str());

    //post processing shadow mapping
    shaders_[PP_SHADOWMAPPING_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                               (shader_path + "pp_shadowmapping.fsh").c_str());

    //post processing sss shader
    shaders_[PP_SUBSURFACE_SCATTERING_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                               (shader_path + "pp_subsurface_scattering.fsh").c_str());

    //post processing mix shader
    shaders_[PP_MIX_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                               (shader_path + "pp_mix.fsh").c_str());

    //post processing finalize shader
    shaders_[PP_FINAL_SHADER] = load_shader((shader_path + "pp_ortho.vsh").c_str(),
                                        (shader_path + "pp_final.fsh").c_str());

    //SHADOWMAP CREATION SHADERS
    //shadow map creation shader for meshes
    shaders_[MESH_SHADOWMAP_SHADER] = load_shader((shader_path + "mesh_shadowmap.vsh").c_str(),
                                         (shader_path + "mesh_shadowmap.fsh").c_str());
    shaders_[SKINNING_SHADOWMAP_SHADER] = load_shader((shader_path + "skinning.vs").c_str(),
                                                      (shader_path + "mesh_shadowmap.fsh").c_str());
    shaders_[SPHERE_SHADOWMAP_SHADER] = load_shader((shader_path + "sphere.vsh").c_str(),
                                         (shader_path + "sphere_shadowmap.fsh").c_str());
    shaders_[CYLINDER_SHADOWMAP_SHADER] = load_shader((shader_path + "cylinder.vsh").c_str(),
                                         (shader_path + "cylinder_shadowmap.fsh").c_str());


    shaders_[SLIDE_SHADER] = load_shader((shader_path + "slide.vs").c_str(),
                                         (shader_path + "slide.fs").c_str());

    // shader for computing the irradiance texture
    shaders_[IRRADIANCE_SHADER] = load_shader((shader_path + "unwrap.vsh").c_str(),
                                              (shader_path + "irradiance.fsh").c_str());

    // final skin shader
    shaders_[FINAL_SKIN_SHADER] = load_shader((shader_path + "phong.vs").c_str(),
                                              (shader_path + "final_skin_shader.fsh").c_str());


    return true;
}

//-----------------------------------------------------------------------------


Shader* GL_state::load_shader(const char *vs_file,
                              const char *fs_file,
                              const char *gs_file)
{
    Shader* shader = new Shader;

    if (shader->load(vs_file, fs_file, gs_file))
    {
        return shader;
    }
    else
    {
        std::cerr << "glState::compileShader: ERROR: Could not compile Shader with filenames \""
                  << vs_file << "\" and \"" << fs_file << "\". " << std::endl;
        delete shader;
        return NULL;
    }
}


//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
