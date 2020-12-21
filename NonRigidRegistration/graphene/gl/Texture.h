//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================

#ifndef TEXTURE_GL_H
#define TEXTURE_GL_H

//== INCLUDES =================================================================

#include <graphene/gl/gl_includes.h>
#include <vector>

//== NAMESPACE ================================================================

namespace graphene {
namespace gl {

//=============================================================================

enum Texture_type
{
    TT_COLOR = 0,
    TT_NORMAL,
    TT_SPECULAR
};

class Texture
{
public:
    GLuint id_;
    std::string name_;

    int width_;
    int height_;
    int channels_;

    std::vector<unsigned char> data_;

    bool srgb_;
    GLint filter_;

    Texture();
    Texture(const std::string& name,int width, int height, unsigned char* data);
    ~Texture();

    ///writes deep copy of this texture into out_texture
    void deep_copy(Texture& out_texture);

    bool read(const std::string& filename);
    bool write(const std::string& filename);

private:


    Texture *flip_y();
};


unsigned char& texel(Texture* t, int x, int y, int ch);

//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_GLSTATE_GL_H
//=============================================================================
