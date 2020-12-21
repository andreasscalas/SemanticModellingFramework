//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/gl/Shader.h>

#include <iostream>
#include <fstream>
#include <sstream>


//== NAMESPACE ================================================================


namespace graphene {
namespace gl {


//=============================================================================


Shader* Shader::s_active_ = NULL;


//-----------------------------------------------------------------------------


Shader::Shader()
    : pid_(0), vid_(0), fid_(0), gid_(0)
{
}


//-----------------------------------------------------------------------------


Shader::~Shader()
{
    cleanup();
}


//-----------------------------------------------------------------------------


void Shader::cleanup()
{
    if (pid_) glDeleteProgram(pid_);
    if (vid_) glDeleteShader(vid_);
    if (fid_) glDeleteShader(fid_);
    if (gid_) glDeleteShader(gid_);
}


//-----------------------------------------------------------------------------


bool Shader::load(const char* vfile, const char* ffile, const char* gfile)
{
    // cleanup existing shaders first
    cleanup();


    // create program
    pid_ = glCreateProgram();


    // vertex shader
    vid_ = load_and_compile(vfile, GL_VERTEX_SHADER);
    if (vid_)  glAttachShader(pid_, vid_);


    // fragment shader
    fid_ = load_and_compile(ffile, GL_FRAGMENT_SHADER);
    if (fid_)  glAttachShader(pid_, fid_);


    // geometry shader
    if (gfile)
    {
        gid_ = load_and_compile(gfile, GL_GEOMETRY_SHADER);
        if (gid_)  glAttachShader(pid_, gid_);
    }


    // bind attribute locations (do this before linking!)
    glBindAttribLocation(pid_, attrib_locations::VERTEX, "vertex");
    glBindAttribLocation(pid_, attrib_locations::NORMAL, "normal");
    glBindAttribLocation(pid_, attrib_locations::COLOR, "color");
    glBindAttribLocation(pid_, attrib_locations::RADIUS, "radius");
    glBindAttribLocation(pid_, attrib_locations::DIRECTION, "direction");
    glBindAttribLocation(pid_, attrib_locations::TEXCOORDS, "texcoords");
    glBindAttribLocation(pid_, attrib_locations::TANGENT, "tangent");
    glBindAttribLocation(pid_, attrib_locations::WEIGHT, "weights");
    glBindAttribLocation(pid_, attrib_locations::DEPEND, "depends");
    glBindAttribLocation(pid_, attrib_locations::WEIGHT2, "weights2");
    glBindAttribLocation(pid_, attrib_locations::DEPEND2, "depends2");
    glBindFragDataLocation(pid_, fragdata_locations::COLOR, "out_color");
    glBindFragDataLocation(pid_, fragdata_locations::COLOR_ALT, "out_color_alt");
    glBindFragDataLocation(pid_, fragdata_locations::NORMAL, "out_normal");



    // link program
    glLinkProgram(pid_);
    GLint status;
    glGetProgramiv(pid_, GL_LINK_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint length;
        glGetProgramiv(pid_, GL_INFO_LOG_LENGTH, &length);

        GLchar *info = new GLchar[length+1];
        glGetProgramInfoLog(pid_, length, NULL, info);
        std::cerr << "Shader: Cannot link program:\n" << info << std::endl;
        delete[] info;

        cleanup();

        return false;
    }


    return true;
}


//-----------------------------------------------------------------------------


GLint Shader::load_and_compile(const char* filename, GLenum type)
{
    // read file to string
    std::ifstream  ifs(filename);
    if (!ifs)
    {
        std::cerr << "Shader: Cannot open file \""  << filename << "\"\n";
        return 0;
    }


    // prepend run-time-dependent version string to shader source
    std::stringstream  ss;
    ss << "#version " << glsl_version() << std::endl;
    ss << ifs.rdbuf();
    std::string str = ss.str();
    const char* source = str.c_str();
    ifs.close();


    // create shader
    GLint id = glCreateShader(type);
    if (!id)
    {
        std::cerr << "Shader: Cannot create shader object for \""  << filename << "\"\n";
        return 0;
    }


    // compile vertex shader
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);


    // check compile status
    GLint status;
    glGetShaderiv(id, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);

        GLchar *info = new GLchar[length+1];
        glGetShaderInfoLog(id, length, NULL, info);

        std::cerr << "Shader: Cannot compile shader \""  << filename << "\"\n" << info << std::endl;

        delete[] info;
        glDeleteShader(id);

        return 0;
    }


    return id;
}


//-----------------------------------------------------------------------------


void Shader::use()
{
    if (pid_) glUseProgram(pid_);
    s_active_ = this;
}


//-----------------------------------------------------------------------------


Shader* Shader::get_active()
{
    return s_active_;
}


//-----------------------------------------------------------------------------


void Shader::disable()
{
    glUseProgram(0);
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, float value)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1f(location, value);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, int value)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1i(location, value);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, unsigned int value)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1ui(location, value);
    return true;
}

//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, const Vec2f &vec)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform2f(location, vec[0], vec[1]);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, const Vec3f& vec)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform3f(location, vec[0], vec[1], vec[2]);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, const Vec4f& vec)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform4f(location, vec[0], vec[1], vec[2], vec[3]);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, const Mat3f& mat)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniformMatrix3fv(location, 1, false, mat.data());
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char* name, const Mat4f &mat)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniformMatrix4fv(location, 1, false, mat.data());
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char *name, const std::vector<Vec3f> &vector)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform3fv(location, vector.size(), vector[0].data());
    return true;
}

//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char *name, const std::vector<float> &vector)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1fv(location, vector.size(), &vector[0]);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char *name, const std::vector<unsigned int> &vector)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1uiv(location, vector.size(), &vector[0]);
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::set_uniform(const char *name, const std::vector<int> &vector)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniform1iv(location, vector.size(), &vector[0]);
    return true;
}

//-----------------------------------------------------------------------------

bool Shader::set_uniform(const char *name, const std::vector<Mat4f> &matrices)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glUniformMatrix4fv(location, matrices.size(), GL_FALSE, matrices[0].data());
    return true;
}

//-----------------------------------------------------------------------------


bool Shader::get_uniform(const char *name, Vec3f &inout_vec)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glGetUniformfv(pid_, location, inout_vec.data());
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::get_uniform(const char *name, Vec4f &inout_vec)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glGetUniformfv(pid_, location, inout_vec.data());
    return true;
}


//-----------------------------------------------------------------------------


bool Shader::get_uniform(const char *name, float &inout_value)
{
    if (!pid_) return false;
    int location = glGetUniformLocation(pid_, name);
    if (location == -1)
    {
        std::cerr << "Shader:set_uniform failed for " << name << std::endl;
        return false;
    }
    glGetUniformfv(pid_, location, &inout_value);
    return true;
}


//-----------------------------------------------------------------------------

bool Shader::bind_texture(const char *sampler_name, GLuint texture_id, GLenum target, GLint texture_unit)
{
    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(target, texture_id);

    GLint id = glGetUniformLocation(pid_, sampler_name);
    if (id == -1)
    {
        std::cerr << "Error: Invalid texture name \"" << sampler_name <<"\"!\n";
        return false;
    }
    
    glUniform1i(id, texture_unit);
    glActiveTexture(GL_TEXTURE0);
    
    return true;
}

//-----------------------------------------------------------------------------

bool Shader::bind_ubo(const char *name, GLuint ubo_id, unsigned int index)
{
    GLuint block_index_ = glGetUniformBlockIndex(pid_, name);

    if (block_index_ == GL_INVALID_INDEX)
    {
        std::cerr << "Shader:bind_ubo failed for: " << name << std::endl;
        return false;
    }

    glUniformBlockBinding(pid_, block_index_, index);
    glBindBufferBase(GL_UNIFORM_BUFFER, index, ubo_id);
    return true;
}

//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
