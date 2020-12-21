//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_SHADER_H
#define GRAPHENE_SHADER_H
//=============================================================================


#include <graphene/geometry/Matrix3x3.h>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/geometry/Vector.h>
#include <graphene/gl/gl_includes.h>
#include <graphene/types.h>

#include <vector>


//=============================================================================


namespace graphene {
namespace gl {


//=============================================================================
namespace attrib_locations {
//=============================================================================


/// enum for attribut locations in shader programs
enum Attrib_location
{
    VERTEX = 0,
    NORMAL,
    COLOR,
    RADIUS,
    DIRECTION,
    TEXCOORDS,
    TANGENT,
    WEIGHT,
    DEPEND,
    WEIGHT2,
    DEPEND2
};


//=============================================================================
} // namespace attrib_locations
//=============================================================================


//=============================================================================
namespace fragdata_locations {
//=============================================================================


/// enum for fragdata locations in shader programs
enum Fragdata_location
{
    COLOR = 0,
    COLOR_ALT,
    NORMAL,
    MAX
};


//=============================================================================
} // namespace fragdata_locations
//=============================================================================


/// A class that makes it easy to compile and link shader programs and also
/// upload uniforms.
class Shader
{

public:

    /// constructor
    Shader();
    /// destructor
    ~Shader();


    /// load, compile, and link vertex/fragment/geometry shader
    bool load(const char* vfile, const char* ffile, const char* gfile=NULL);


    /// enable or disable the use of this shader program
    void use();
    void disable();


    /// upload uniforms
    bool set_uniform(const char* name, float value);
    bool set_uniform(const char* name, int value);
    bool set_uniform(const char* name, unsigned int value);
    bool set_uniform(const char* name, const Vec2f& vec);
    bool set_uniform(const char* name, const Vec3f& vec);
    bool set_uniform(const char* name, const Vec4f& vec);
    bool set_uniform(const char* name, const Mat3f& mat);
    bool set_uniform(const char* name, const Mat4f& mat);
    bool set_uniform(const char* name, const std::vector<Vec3f>& vector);
    bool set_uniform(const char* name, const std::vector<float>& vector);
    bool set_uniform(const char* name, const std::vector<unsigned int>& vector);
    bool set_uniform(const char* name, const std::vector<int>& vector);
    bool set_uniform(const char *name, const std::vector<Mat4f> &matrices);


    /// get uniforms
    bool get_uniform(const char* name, Vec3f& inout_vec);
    bool get_uniform(const char* name, Vec4f& inout_vec);
    bool get_uniform(const char* name, float &inout_value);

    /// bind texture
    bool bind_texture(const char* sampler_name, GLuint texture_id, GLenum target, GLint texture_unit);

    ///bind uniform buffer (shader must be active)
    bool bind_ubo(const char* name, GLuint ubo_id, unsigned int index);

    /// return id of the shader
    GLuint get_shaderid() {return pid_;}

    /// returns pointer to currently active shader
    static Shader* get_active();


private:

    void  cleanup();
    GLint load_and_compile(const char* filename, GLenum type);


private:

    GLuint pid_, vid_, fid_, gid_;
    static Shader* s_active_;


};


//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SHADER_H
//=============================================================================
