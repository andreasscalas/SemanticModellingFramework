//=============================================================================

#ifndef GRAPHENE_GL_H
#define GRAPHENE_GL_H

//== INCLUDES =================================================================

/// \addtogroup gl gl
/// @{

#ifdef __APPLE__
#  include <GL/glew.h>
#elif _WIN32
#  include <stdlib.h>
#  include "GL\glew.h"
#else
#  include <GL/glew.h>
#endif

#include <cstdlib>
#include <iostream>


//=============================================================================


/// Check for OpenGL errors.
inline void glCheckError(const char* _info="")
{
    GLenum error;
    while ((error = glGetError()) != GL_NO_ERROR)
    {
        std::cerr << "GL error at \"" << _info << "\": ";

        switch (error)
        {
            case GL_INVALID_ENUM:
                std::cerr << "invalid enum\n";
                break;

            case GL_INVALID_VALUE:
                std::cerr << "invalid value (out of range)\n";
                break;

            case GL_INVALID_OPERATION:
                std::cerr << "invalid operation (not allowed in current state)\n";
                break;

            case GL_INVALID_FRAMEBUFFER_OPERATION:
                std::cerr << "invalid framebuffer operation (framebuffer not complete)\n";
                break;

            case GL_OUT_OF_MEMORY:
                std::cerr << "out of memory\n";
                break;


            case GL_STACK_UNDERFLOW:
                std::cerr << "stack underflow\n";
                break;

            case GL_STACK_OVERFLOW:
                std::cerr << "stack overflow\n";
                break;
        }

        std::cerr << std::endl;
    }
}


//-----------------------------------------------------------------------------


/// utility function to return the GLSL version number as integer
inline unsigned int glsl_version()
{
    // determine glsl version
    std::string glsl_version((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // strip vendor string
    size_t pos = glsl_version.find(" ");
    if (pos != std::string::npos) glsl_version.replace(pos,glsl_version.size()-pos,"");

    // replace dots
    for (unsigned int i(0); i < 3; i++)
    {
        size_t pos = glsl_version.find(".");
        if (pos != std::string::npos) glsl_version.replace(pos,1,"");
    }

    return atoi(glsl_version.c_str());
}


//=============================================================================
/// @}
//=============================================================================
#endif // GRAPHENE_GL_H
//=============================================================================
